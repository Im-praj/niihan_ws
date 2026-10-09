class Viewer3D {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x111111);
        
        // Camera setup
        const aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 1000);
        // Position camera to look at the ground plane
        this.camera.position.set(3, -4, 3);
        this.camera.up.set(0, 0, 1); // Z is up in ROS
        
        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.container.appendChild(this.renderer.domElement);
        
        // Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.target.set(0, 0, 0);
        
        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
        dirLight.position.set(10, 10, 20);
        this.scene.add(dirLight);
        
        // Grid helper
        const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
        gridHelper.rotation.x = Math.PI / 2; // Orient grid to XY plane
        this.scene.add(gridHelper);
        
        // Axes helper (X=red, Y=green, Z=blue)
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);
        
        // Navigation Plane (invisible, for raycasting)
        const planeGeometry = new THREE.PlaneGeometry(1000, 1000);
        const planeMaterial = new THREE.MeshBasicMaterial({ visible: false });
        this.navPlane = new THREE.Mesh(planeGeometry, planeMaterial);
        this.scene.add(this.navPlane);
        
        // Raycaster
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        
        // Groups
        this.pcGroup = new THREE.Group();
        this.scene.add(this.pcGroup);
        
        this.wpGroup = new THREE.Group();
        this.scene.add(this.wpGroup);
        
        this.pathGroup = new THREE.Group();
        this.scene.add(this.pathGroup);
        
        this.gfGroup = new THREE.Group();
        this.scene.add(this.gfGroup);
        
        // Robot marker
        this.createRobotMarker();
        
        // Resize listener
        window.addEventListener('resize', this.onWindowResize.bind(this));
        
        // Animation loop
        this.animate = this.animate.bind(this);
        this.animate();
        
        this.onClickCallback = null;
        this.pointerStart = null;
        this.pointerDragged = false;
        this.renderer.domElement.addEventListener('pointerdown', (event) => {
            this.pointerStart = {x: event.clientX, y: event.clientY};
            this.pointerDragged = false;
        });
        this.renderer.domElement.addEventListener('pointermove', (event) => {
            if (this.pointerStart && Math.hypot(event.clientX - this.pointerStart.x,
                event.clientY - this.pointerStart.y) > 4) this.pointerDragged = true;
        });
        this.renderer.domElement.addEventListener('pointerup', () => { this.pointerStart = null; });
        this.renderer.domElement.addEventListener('pointercancel', () => {
            this.pointerStart = null;
            this.pointerDragged = true;
        });
        this.renderer.domElement.addEventListener('click', this.onMouseClick.bind(this));
    }

    clearGroup(group) {
        // Removing a Three.js object alone does not release its GPU buffers.
        while (group.children.length) {
            const child = group.children[0];
            child.traverse(object => {
                if (object.geometry) object.geometry.dispose();
                const materials = Array.isArray(object.material) ? object.material : [object.material];
                materials.forEach(material => { if (material) material.dispose(); });
            });
            group.remove(child);
        }
    }
    
    createRobotMarker() {
        this.robotMarker = new THREE.Group();
        
        this.scene.add(this.robotMarker);
    }
    
    updateRobotModel(model) {
        const links = new Map();
        const transform = (group, origin) => {
            group.position.fromArray(origin.xyz);
            group.rotation.set(...origin.rpy, 'ZYX');
        };
        for (const link of model.links) {
            const group = new THREE.Group();
            group.name = link.name;
            for (const visual of link.visuals) {
                const shape = visual.geometry;
                let geometry;
                if (shape.kind === 'box') geometry = new THREE.BoxGeometry(...shape.size);
                else if (shape.kind === 'cylinder') {
                    geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.length, 32);
                    geometry.rotateX(Math.PI / 2); // URDF cylinders extend along Z.
                } else if (shape.kind === 'sphere') geometry = new THREE.SphereGeometry(shape.radius, 24, 16);
                else continue;
                const rgba = visual.rgba;
                const material = new THREE.MeshPhongMaterial({
                    color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
                    opacity: rgba[3], transparent: rgba[3] < 1
                });
                const mesh = new THREE.Mesh(geometry, material);
                transform(mesh, visual);
                group.add(mesh);
            }
            links.set(link.name, group);
        }
        for (const joint of model.joints) {
            const child = links.get(joint.child);
            transform(child, joint);
            links.get(joint.parent).add(child);
        }
        this.clearGroup(this.robotMarker);
        this.robotMarker.add(links.get(model.root));
        this.robotMarker.name = model.name;
    }

    updateRobotPose(x, y, z, yaw) {
        this.robotMarker.position.set(x, y, z);
        this.robotMarker.rotation.z = yaw;
    }
    
    updatePointCloud(data) {
        // data is flat array of floats [x,y,z, x,y,z...]
        // Clear previous
        this.clearGroup(this.pcGroup);
        
        const geometry = new THREE.BufferGeometry();
        const vertices = new Float32Array(data);
        geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
        
        // Color based on Z height
        const colors = new Float32Array(data.length);
        const color = new THREE.Color();
        for (let i = 0; i < data.length; i += 3) {
            const z = data[i+2];
            // Simple color map based on Z
            const h = (1.0 - Math.min(Math.max(z / 3.0, 0.0), 1.0)) * 0.7; 
            color.setHSL(h, 1.0, 0.5);
            colors[i] = color.r;
            colors[i+1] = color.g;
            colors[i+2] = color.b;
        }
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        
        const material = new THREE.PointsMaterial({ size: 0.05, vertexColors: true });
        const points = new THREE.Points(geometry, material);
        this.pcGroup.add(points);
    }
    
    updateWaypoints(waypoints) {
        this.clearGroup(this.wpGroup);
        
        waypoints.forEach(wp => {
            const group = new THREE.Group();
            group.position.set(wp.x, wp.y, wp.z);
            
            // Sphere
            const color = wp.status === "ACTIVE" ? 0xffff00 : (wp.status === "COMPLETED" ? 0x00ff00 :
                (['FAILED', 'MISSED'].includes(wp.status) ? 0xff4444 : 0x00bcd4));
            const sphGeo = new THREE.SphereGeometry(0.07, 16, 16);
            const sphMat = new THREE.MeshPhongMaterial({ color: color });
            const sphere = new THREE.Mesh(sphGeo, sphMat);
            group.add(sphere);
            
            this.wpGroup.add(group);
        });
        
        this.updatePath(waypoints);
    }
    
    updatePath(waypoints) {
        this.clearGroup(this.pathGroup);
        
        if (waypoints.length < 2) return;
        
        const points = [];
        waypoints.forEach(wp => {
            points.push(new THREE.Vector3(wp.x, wp.y, wp.z));
        });
        
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0xffa500, linewidth: 2 });
        const line = new THREE.Line(geometry, material);
        this.pathGroup.add(line);
    }
    
    updateGeofence(polygon) {
        this.clearGroup(this.gfGroup);
        
        if (!polygon || polygon.length < 3) return;
        
        // Draw polygon line
        const points = [];
        polygon.forEach(p => {
            points.push(new THREE.Vector3(p.x, p.y, 0));
        });
        points.push(new THREE.Vector3(polygon[0].x, polygon[0].y, 0)); // close loop
        
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0xff00ff, linewidth: 3 });
        const line = new THREE.Line(geometry, material);
        
        // Add translucent polygon area
        const shape = new THREE.Shape();
        shape.moveTo(polygon[0].x, polygon[0].y);
        for(let i = 1; i < polygon.length; i++) {
            shape.lineTo(polygon[i].x, polygon[i].y);
        }
        
        const shapeGeo = new THREE.ShapeGeometry(shape);
        const shapeMat = new THREE.MeshBasicMaterial({ color: 0xff00ff, transparent: true, opacity: 0.1, side: THREE.DoubleSide });
        const mesh = new THREE.Mesh(shapeGeo, shapeMat);
        // mesh lies on XY plane, need to ensure it's slightly above z=0 if needed, but keeping at z=0 is fine
        
        this.gfGroup.add(line);
        this.gfGroup.add(mesh);
    }
    
    onMouseClick(event) {
        // Orbit/pan gestures must not submit a goal at their release position.
        if (this.pointerDragged) { this.pointerDragged = false; return; }
        if (!this.onClickCallback) return;
        
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Intersect with nav plane
        const intersects = this.raycaster.intersectObject(this.navPlane);
        
        if (intersects.length > 0) {
            const point = intersects[0].point;
            this.onClickCallback(point.x, point.y, point.z);
        }
    }
    
    setOnClickCallback(cb) {
        this.onClickCallback = cb;
    }
    
    onWindowResize() {
        if (!this.container || !this.container.clientWidth || !this.container.clientHeight) return;
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    animate() {
        requestAnimationFrame(this.animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}
