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
        this.camera.position.set(0, 10, 10);
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
        this.renderer.domElement.addEventListener('click', this.onMouseClick.bind(this));
    }
    
    createRobotMarker() {
        this.robotMarker = new THREE.Group();
        
        // Body (approx 0.7 x 0.5 x 0.3)
        const bodyGeo = new THREE.BoxGeometry(0.7, 0.5, 0.3);
        const bodyMat = new THREE.MeshPhongMaterial({ color: 0x00bcd4, transparent: true, opacity: 0.8 });
        const body = new THREE.Mesh(bodyGeo, bodyMat);
        body.position.z = 0.15;
        this.robotMarker.add(body);
        
        // Wheels
        const wheelGeo = new THREE.CylinderGeometry(0.1, 0.1, 0.1, 16);
        const wheelMat = new THREE.MeshPhongMaterial({ color: 0x333333 });
        
        const positions = [
            [0.2, 0.3, 0.1], [0.2, -0.3, 0.1],
            [-0.2, 0.3, 0.1], [-0.2, -0.3, 0.1]
        ];
        
        positions.forEach(pos => {
            const wheel = new THREE.Mesh(wheelGeo, wheelMat);
            wheel.position.set(pos[0], pos[1], pos[2]);
            wheel.rotation.x = Math.PI / 2;
            this.robotMarker.add(wheel);
        });
        
        // Direction arrow
        const dirGeo = new THREE.ConeGeometry(0.1, 0.2, 16);
        const dirMat = new THREE.MeshPhongMaterial({ color: 0xff0000 });
        const dir = new THREE.Mesh(dirGeo, dirMat);
        dir.rotation.z = -Math.PI / 2; // point along X axis
        dir.position.set(0.4, 0, 0.3);
        this.robotMarker.add(dir);
        
        this.scene.add(this.robotMarker);
    }
    
    updateRobotPose(x, y, z, yaw) {
        this.robotMarker.position.set(x, y, z);
        this.robotMarker.rotation.z = yaw;
    }
    
    updatePointCloud(data) {
        // data is flat array of floats [x,y,z, x,y,z...]
        // Clear previous
        while(this.pcGroup.children.length > 0) { 
            this.pcGroup.remove(this.pcGroup.children[0]); 
        }
        
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
        while(this.wpGroup.children.length > 0) {
            this.wpGroup.remove(this.wpGroup.children[0]);
        }
        
        waypoints.forEach(wp => {
            const group = new THREE.Group();
            group.position.set(wp.x, wp.y, wp.z);
            group.rotation.z = wp.yaw;
            
            // Sphere
            const color = wp.status === "ACTIVE" ? 0xffff00 : (wp.status === "COMPLETED" ? 0x00ff00 : 0x00bcd4);
            const sphGeo = new THREE.SphereGeometry(0.2, 16, 16);
            const sphMat = new THREE.MeshPhongMaterial({ color: color });
            const sphere = new THREE.Mesh(sphGeo, sphMat);
            group.add(sphere);
            
            // Arrow pointing in yaw direction (X axis)
            const arrGeo = new THREE.ConeGeometry(0.1, 0.3, 16);
            const arrMat = new THREE.MeshPhongMaterial({ color: 0xff0000 });
            const arrow = new THREE.Mesh(arrGeo, arrMat);
            arrow.rotation.z = -Math.PI / 2;
            arrow.position.x = 0.3;
            group.add(arrow);
            
            this.wpGroup.add(group);
        });
        
        this.updatePath(waypoints);
    }
    
    updatePath(waypoints) {
        while(this.pathGroup.children.length > 0) {
            this.pathGroup.remove(this.pathGroup.children[0]);
        }
        
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
        while(this.gfGroup.children.length > 0) {
            this.gfGroup.remove(this.gfGroup.children[0]);
        }
        
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
        if (!this.container) return;
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
