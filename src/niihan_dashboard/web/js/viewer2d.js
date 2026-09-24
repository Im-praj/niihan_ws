class Viewer2D {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.mapData = null;
        this.robotPose = null;
        this.waypoints = [];
        this.geofence = [];
        
        // Simple scaling
        this.scale = 10.0; // pixels per meter
        this.offsetX = this.canvas.width / 2;
        this.offsetY = this.canvas.height / 2;
        this.onClickCallback = null;
        this.canvas.addEventListener('click', (e) => this.handleClick(e));

        this.requestRender();
    }

    setOnClickCallback(cb) {
        this.onClickCallback = cb;
    }

    updateMap(mapData) {
        this.mapData = mapData;
        this.requestRender();
    }

    updateRobotPose(x, y, yaw) {
        this.robotPose = {x, y, yaw};
        this.requestRender();
    }

    updateWaypoints(waypoints) {
        this.waypoints = waypoints;
        this.requestRender();
    }

    updateGeofence(geofence) {
        this.geofence = geofence;
        this.requestRender();
    }

    requestRender() {
        if (!this.renderPending) {
            this.renderPending = true;
            requestAnimationFrame(() => this.render());
        }
    }

    worldToCanvas(x, y) {
        // If we have a map, maybe center around it? 
        // Or just center around the robot. Let's center around robot pose if available, else 0,0
        let cx = 0; let cy = 0;
        if (this.robotPose) {
            cx = this.robotPose.x;
            cy = this.robotPose.y;
        }
        
        const px = this.canvas.width / 2 + (x - cx) * this.scale;
        const py = this.canvas.height / 2 - (y - cy) * this.scale; // Y is up in world, down in canvas
        return {x: px, y: py};
    }

    canvasToWorld(px, py) {
        let cx = 0; let cy = 0;
        if (this.robotPose) {
            cx = this.robotPose.x;
            cy = this.robotPose.y;
        }
        const x = cx + (px - this.canvas.width / 2) / this.scale;
        const y = cy - (py - this.canvas.height / 2) / this.scale;
        return {x, y};
    }

    handleClick(e) {
        if (!this.onClickCallback) return;
        const rect = this.canvas.getBoundingClientRect();
        const px = e.clientX - rect.left;
        const py = e.clientY - rect.top;
        const world = this.canvasToWorld(px, py);
        this.onClickCallback(world.x, world.y, 0);
    }

    render() {
        this.renderPending = false;
        
        // Clear
        this.ctx.fillStyle = '#1e1e1e';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw Map
        if (this.mapData) {
            this.drawMap();
        }

        // Draw Geofence
        if (this.geofence && this.geofence.length > 0) {
            this.drawGeofence();
        }

        // Draw Waypoints
        if (this.waypoints && this.waypoints.length > 0) {
            this.drawWaypoints();
        }

        // Draw Robot
        if (this.robotPose) {
            this.drawRobot();
        }
    }

    drawMap() {
        // Drawing a full occupancy grid to canvas pixel by pixel in JS can be slow.
        // We will draw it as an ImageData or just draw the occupied cells.
        // For simplicity and speed, let's draw only obstacles (100).
        this.ctx.fillStyle = '#aaaaaa';
        const w = this.mapData.width;
        const h = this.mapData.height;
        const res = this.mapData.resolution;
        const ox = this.mapData.origin.x;
        const oy = this.mapData.origin.y;
        
        // Size of one cell in pixels
        const cellSize = res * this.scale;

        this.ctx.beginPath();
        for (let y = 0; y < h; y++) {
            for (let x = 0; x < w; x++) {
                const idx = y * w + x;
                const val = this.mapData.data[idx];
                if (val > 50) {
                    const wx = ox + x * res;
                    const wy = oy + y * res;
                    const p = this.worldToCanvas(wx, wy);
                    this.ctx.rect(p.x, p.y - cellSize, cellSize, cellSize);
                }
            }
        }
        this.ctx.fill();
    }

    drawGeofence() {
        this.ctx.strokeStyle = 'red';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        for (let i = 0; i < this.geofence.length; i++) {
            const p = this.worldToCanvas(this.geofence[i].x, this.geofence[i].y);
            if (i === 0) this.ctx.moveTo(p.x, p.y);
            else this.ctx.lineTo(p.x, p.y);
        }
        this.ctx.closePath();
        this.ctx.stroke();
    }

    drawWaypoints() {
        this.ctx.strokeStyle = '#00ff00';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        
        let prev = null;
        for (let wp of this.waypoints) {
            const p = this.worldToCanvas(wp.x, wp.y);
            
            // Draw line
            if (prev) {
                this.ctx.moveTo(prev.x, prev.y);
                this.ctx.lineTo(p.x, p.y);
            }
            prev = p;
        }
        this.ctx.stroke();

        for (let wp of this.waypoints) {
            const p = this.worldToCanvas(wp.x, wp.y);
            this.ctx.fillStyle = wp.status === 'COMPLETED' ? '#008800' : (wp.status === 'ACTIVE' ? '#ffff00' : '#00ff00');
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // Draw heading
            this.ctx.strokeStyle = '#ffffff';
            this.ctx.beginPath();
            this.ctx.moveTo(p.x, p.y);
            this.ctx.lineTo(p.x + Math.cos(-wp.yaw) * 15, p.y + Math.sin(-wp.yaw) * 15);
            this.ctx.stroke();
        }
    }

    drawRobot() {
        const p = this.worldToCanvas(this.robotPose.x, this.robotPose.y);
        
        this.ctx.fillStyle = '#0088ff';
        this.ctx.beginPath();
        this.ctx.arc(p.x, p.y, 8, 0, 2 * Math.PI);
        this.ctx.fill();
        
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(p.x, p.y);
        // Canvas Y is inverted
        this.ctx.lineTo(p.x + Math.cos(-this.robotPose.yaw) * 20, p.y + Math.sin(-this.robotPose.yaw) * 20);
        this.ctx.stroke();
    }
}
