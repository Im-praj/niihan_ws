class Viewer2D {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.mapData = null;
        this.mapImage = null;
        this.robotPose = null;
        this.waypoints = [];
        this.geofence = [];
        this.center = {x: 0, y: 0};
        this.scale = 10; // backing-canvas pixels per meter
        this.pixelRatio = 1;
        this.autoFit = true;
        this.onClickCallback = null;
        this.dragStart = null;
        this.dragged = false;
        this.canvas.addEventListener('click', (event) => this.handleClick(event));
        this.canvas.addEventListener('wheel', (event) => this.handleZoom(event), {passive: false});
        this.canvas.addEventListener('pointerdown', (event) => {
            if (event.button !== 0) return;
            this.dragStart = {point: this.eventPoint(event), center: {...this.center}};
            this.dragged = false;
            this.canvas.setPointerCapture(event.pointerId);
        });
        this.canvas.addEventListener('pointermove', (event) => {
            if (!this.dragStart) return;
            const point = this.eventPoint(event);
            const dx = point.x - this.dragStart.point.x;
            const dy = point.y - this.dragStart.point.y;
            if (Math.hypot(dx, dy) > 4 * this.pixelRatio) this.dragged = true;
            if (!this.dragged) return;
            this.autoFit = false;
            this.center = {
                x: this.dragStart.center.x - dx / this.scale,
                y: this.dragStart.center.y + dy / this.scale
            };
            this.requestRender();
        });
        this.canvas.addEventListener('pointerup', () => { this.dragStart = null; });
        this.canvas.addEventListener('pointercancel', () => {
            this.dragStart = null;
            this.dragged = true;
        });
        window.addEventListener('resize', () => this.onWindowResize());
        if (typeof ResizeObserver !== 'undefined') {
            this.resizeObserver = new ResizeObserver(() => this.onWindowResize());
            this.resizeObserver.observe(this.canvas.parentElement);
        }
        this.onWindowResize();
        this.requestRender();
    }

    setOnClickCallback(callback) {
        this.onClickCallback = callback;
    }

    updateMap(mapData) {
        const {width, height, resolution, origin, data} = mapData;
        if (!Number.isSafeInteger(width) || width <= 0 ||
            !Number.isSafeInteger(height) || height <= 0 ||
            !Number.isFinite(resolution) || resolution <= 0 ||
            !origin || !Number.isFinite(origin.x) || !Number.isFinite(origin.y) ||
            !Number.isFinite((origin.yaw === undefined ? 0 : origin.yaw)) || !data || data.length !== width * height) {
            return false;
        }
        this.mapData = mapData;
        // Rasterize once per map message, rather than traversing every grid cell
        // again for each pose update. Row zero is local map y=0.
        this.mapImage = document.createElement('canvas');
        this.mapImage.width = width;
        this.mapImage.height = height;
        const context = this.mapImage.getContext('2d');
        const image = context.createImageData(width, height);
        for (let index = 0; index < data.length; index++) {
            const value = data[index];
            const shade = value < 0 ? 80 : Math.round(235 - 210 * Math.min(value, 100) / 100);
            image.data[index * 4] = shade;
            image.data[index * 4 + 1] = shade;
            image.data[index * 4 + 2] = shade;
            image.data[index * 4 + 3] = 255;
        }
        context.putImageData(image, 0, 0);
        if (this.autoFit) this.fitMap();
        this.requestRender();
        return true;
    }

    updateRobotPose(x, y, yaw) {
        if (![x, y, yaw].every(Number.isFinite)) return;
        this.robotPose = {x, y, yaw};
        if (!this.mapData && this.autoFit) this.center = {x, y};
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

    onWindowResize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        // The 2D tab starts hidden. Keep its backing buffer until it is visible.
        if (!rect.width || !rect.height) return;
        const ratio = window.devicePixelRatio || 1;
        const width = Math.max(1, Math.round(rect.width * ratio));
        const height = Math.max(1, Math.round(rect.height * ratio));
        if (width === this.canvas.width && height === this.canvas.height && ratio === this.pixelRatio) return;
        this.scale *= ratio / this.pixelRatio;
        this.pixelRatio = ratio;
        this.canvas.width = width;
        this.canvas.height = height;
        if (this.autoFit) this.fitMap();
        this.requestRender();
    }

    mapToWorld(x, y) {
        const {origin} = this.mapData;
        const yaw = (origin.yaw === undefined ? 0 : origin.yaw);
        return {
            x: origin.x + Math.cos(yaw) * x - Math.sin(yaw) * y,
            y: origin.y + Math.sin(yaw) * x + Math.cos(yaw) * y
        };
    }

    fitMap() {
        this.autoFit = true;
        if (this.mapData) {
            const width = this.mapData.width * this.mapData.resolution;
            const height = this.mapData.height * this.mapData.resolution;
            const corners = [[0, 0], [width, 0], [0, height], [width, height]]
                .map(([x, y]) => this.mapToWorld(x, y));
            const minX = Math.min(...corners.map(point => point.x));
            const maxX = Math.max(...corners.map(point => point.x));
            const minY = Math.min(...corners.map(point => point.y));
            const maxY = Math.max(...corners.map(point => point.y));
            this.center = {x: (minX + maxX) / 2, y: (minY + maxY) / 2};
            this.scale = 0.9 * Math.min(this.canvas.width / (maxX - minX), this.canvas.height / (maxY - minY));
        } else if (this.robotPose) {
            this.center = {x: this.robotPose.x, y: this.robotPose.y};
        }
        this.requestRender();
    }

    requestRender() {
        if (!this.renderPending) {
            this.renderPending = true;
            requestAnimationFrame(() => this.render());
        }
    }

    worldToCanvas(x, y) {
        return {
            x: this.canvas.width / 2 + (x - this.center.x) * this.scale,
            y: this.canvas.height / 2 - (y - this.center.y) * this.scale
        };
    }

    canvasToWorld(x, y) {
        return {
            x: this.center.x + (x - this.canvas.width / 2) / this.scale,
            y: this.center.y - (y - this.canvas.height / 2) / this.scale
        };
    }

    eventPoint(event) {
        const rect = this.canvas.getBoundingClientRect();
        return {
            x: (event.clientX - rect.left) * this.canvas.width / rect.width,
            y: (event.clientY - rect.top) * this.canvas.height / rect.height
        };
    }

    handleClick(event) {
        if (this.dragged) { this.dragged = false; return; }
        if (!this.onClickCallback) return;
        const point = this.eventPoint(event);
        const world = this.canvasToWorld(point.x, point.y);
        if (Number.isFinite(world.x) && Number.isFinite(world.y)) {
            this.onClickCallback(world.x, world.y, 0);
        }
    }

    handleZoom(event) {
        event.preventDefault();
        const point = this.eventPoint(event);
        const before = this.canvasToWorld(point.x, point.y);
        this.scale = Math.min(10000 * this.pixelRatio, Math.max(0.1 * this.pixelRatio,
            this.scale * Math.exp(-Math.max(-100, Math.min(100, event.deltaY)) * 0.002)));
        const after = this.canvasToWorld(point.x, point.y);
        this.center.x += before.x - after.x;
        this.center.y += before.y - after.y;
        this.autoFit = false;
        this.requestRender();
    }

    render() {
        this.renderPending = false;
        this.ctx.fillStyle = '#1e1e1e';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        if (this.mapData) this.drawMap();
        if (this.geofence.length) this.drawGeofence();
        if (this.waypoints.length) this.drawWaypoints();
        if (this.robotPose) this.drawRobot();
    }

    drawMap() {
        const origin = this.worldToCanvas(this.mapData.origin.x, this.mapData.origin.y);
        const cellSize = this.mapData.resolution * this.scale;
        this.ctx.save();
        this.ctx.translate(origin.x, origin.y);
        this.ctx.rotate(-(this.mapData.origin.yaw === undefined ? 0 : this.mapData.origin.yaw));
        this.ctx.scale(cellSize, -cellSize);
        this.ctx.imageSmoothingEnabled = false;
        this.ctx.drawImage(this.mapImage, 0, 0);
        this.ctx.restore();
    }

    drawGeofence() {
        this.ctx.strokeStyle = '#ff33cc';
        this.ctx.lineWidth = 2 * this.pixelRatio;
        this.ctx.beginPath();
        this.geofence.forEach((point, index) => {
            const p = this.worldToCanvas(point.x, point.y);
            if (index === 0) this.ctx.moveTo(p.x, p.y);
            else this.ctx.lineTo(p.x, p.y);
        });
        if (this.geofence.length >= 3) this.ctx.closePath();
        this.ctx.stroke();
        for (const point of this.geofence) {
            const p = this.worldToCanvas(point.x, point.y);
            this.ctx.fillStyle = '#ff33cc';
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, 3 * this.pixelRatio, 0, 2 * Math.PI);
            this.ctx.fill();
        }
    }

    drawWaypoints() {
        this.ctx.strokeStyle = '#00cc55';
        this.ctx.lineWidth = 2 * this.pixelRatio;
        // These lines indicate mission order, not an obstacle-free Nav2 path.
        this.ctx.setLineDash([5 * this.pixelRatio, 5 * this.pixelRatio]);
        this.ctx.beginPath();
        this.waypoints.forEach((waypoint, index) => {
            const p = this.worldToCanvas(waypoint.x, waypoint.y);
            if (index === 0) this.ctx.moveTo(p.x, p.y);
            else this.ctx.lineTo(p.x, p.y);
        });
        this.ctx.stroke();
        this.ctx.setLineDash([]);
        for (const waypoint of this.waypoints) {
            const p = this.worldToCanvas(waypoint.x, waypoint.y);
            this.ctx.fillStyle = waypoint.status === 'COMPLETED' ? '#008800' :
                (waypoint.status === 'ACTIVE' ? '#ffff00' :
                (['FAILED', 'MISSED'].includes(waypoint.status) ? '#ff4444' : '#00dd55'));
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, 5 * this.pixelRatio, 0, 2 * Math.PI);
            this.ctx.fill();

        }
    }

    drawRobot() {
        const p = this.worldToCanvas(this.robotPose.x, this.robotPose.y);
        this.ctx.fillStyle = '#0088ff';
        this.ctx.beginPath();
        this.ctx.arc(p.x, p.y, 8 * this.pixelRatio, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2 * this.pixelRatio;
        this.ctx.beginPath();
        this.ctx.moveTo(p.x, p.y);
        this.ctx.lineTo(p.x + Math.cos(this.robotPose.yaw) * 20 * this.pixelRatio,
            p.y - Math.sin(this.robotPose.yaw) * 20 * this.pixelRatio);
        this.ctx.stroke();
    }
}

if (typeof module !== 'undefined' && module.exports) module.exports = Viewer2D;
