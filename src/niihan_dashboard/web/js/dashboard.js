let ws = null;
let isConnected = false;
let mode = "MANUAL";
let estopActive = false;
let nav2Ready = false;

// DOM Elements
const elConn = document.getElementById('conn-status');
const elMode = document.getElementById('mode-status');
const elLoc = document.getElementById('loc-status');
const elNav = document.getElementById('nav-status');
const btnEstop = document.getElementById('btn-estop');
const btnClearEstop = document.getElementById('btn-clear-estop');
const btnManual = document.getElementById('btn-mode-manual');
const btnAuto = document.getElementById('btn-mode-auto');
const camStream = document.getElementById('camera-stream');
const wpList = document.getElementById('wp-list');
const missionState = document.getElementById('mission-state');
const missionProgress = document.getElementById('mission-progress');
const gfState = document.getElementById('gf-state');

// State
let robotPose = {x: 0, y: 0, z: 0, yaw: 0};
let mapData = null;
let waypoints = [];
let geofencePolygon = [];

// Interactions
let interactionMode = "NONE"; // NONE, ADD_WP, SET_GOAL, CREATE_GF
let tempPolygon = [];

// 3D Viewer
let viewer3d = null;

window.addEventListener('load', () => {
    viewer3d = new Viewer3D('three-canvas-container');
    viewer3d.setOnClickCallback(onMapClick);
    
    // Tab switching
    const tabBtns = document.querySelectorAll('.tab-btn');
    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.querySelectorAll('.view-container').forEach(c => c.classList.remove('active'));
            
            btn.classList.add('active');
            const target = document.getElementById(btn.dataset.target);
            target.classList.add('active');
            
            if (btn.dataset.target === 'view-3d') {
                viewer3d.onWindowResize();
            }
        });
    });
    connectWebSocket();
});

function connectWebSocket() {
    ws = new WebSocket(`ws://${window.location.hostname}:8081`);

    ws.onopen = () => {
        isConnected = true;
        elConn.textContent = "CONNECTED";
        elConn.className = "status-badge ok";
    };

    ws.onclose = () => {
        isConnected = false;
        elConn.textContent = "DISCONNECTED";
        elConn.className = "status-badge error";
        setTimeout(connectWebSocket, 2000);
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        if (data.type === "telemetry") {
            updateTelemetry(data);
        } else if (data.type === "map") {
            mapData = data;
            document.getElementById('map-status-3d').textContent = `MAP SOURCE: SLAM | STATUS: READY | Res: ${data.resolution}m`;
            // In a real implementation, convert occupancy grid to 3D mesh or use pointcloud. We will rely on pointcloud.
        } else if (data.type === "pointcloud") {
            if (viewer3d) viewer3d.updatePointCloud(data.data);
        } else if (data.type === "camera") {
            camStream.src = "data:image/jpeg;base64," + data.image;
        } else if (data.type === "mission_write_response") {
            alert(data.message);
            if (data.success) {
                document.getElementById('btn-start-mission').disabled = false;
            }
        }
    };
}

function updateTelemetry(data) {
    // Pose
    document.getElementById('tel-x').textContent = data.pose.x.toFixed(2);
    document.getElementById('tel-y').textContent = data.pose.y.toFixed(2);
    document.getElementById('tel-z').textContent = data.pose.z.toFixed(2);
    document.getElementById('tel-yaw').textContent = (data.pose.yaw * 180 / Math.PI).toFixed(1);
    
    // Vel
    document.getElementById('tel-vel-x').textContent = data.velocity.linear_x.toFixed(2);
    document.getElementById('tel-vel-z').textContent = data.velocity.angular_z.toFixed(2);
    
    // Status
    elLoc.textContent = `LOC: ${data.localization.source}`;
    
    mode = data.mode;
    if(mode === "MANUAL") {
        btnManual.classList.add('active');
        btnAuto.classList.remove('active');
        elMode.textContent = "MANUAL";
    } else {
        btnAuto.classList.add('active');
        btnManual.classList.remove('active');
        elMode.textContent = "AUTO";
    }
    
    estopActive = data.estop;
    if(estopActive) {
        btnEstop.style.display = "none";
        btnClearEstop.style.display = "block";
        document.getElementById('btn-start-mission').disabled = true;
    } else {
        btnEstop.style.display = "block";
        btnClearEstop.style.display = "none";
    }
    
    nav2Ready = data.nav2_ready;
    elNav.textContent = nav2Ready ? "NAV2: READY" : "NAV2: NOT AVAILABLE";
    elNav.className = nav2Ready ? "status-badge ok" : "status-badge error";
    
    robotPose = data.pose;
    if (viewer3d) viewer3d.updateRobotPose(robotPose.x, robotPose.y, robotPose.z, robotPose.yaw);
    
    // Mission
    const m = data.mission;
    missionState.textContent = m.state;
    waypoints = m.waypoints;
    if (viewer3d) viewer3d.updateWaypoints(waypoints);
    renderWaypointTable();
    
    // Geofence
    const gf = data.geofence;
    if (gf.enabled) {
        gfState.textContent = `ARMED (${gf.vertices} pts)`;
        geofencePolygon = gf.polygon;
        if (viewer3d) viewer3d.updateGeofence(geofencePolygon);
    } else {
        gfState.textContent = "DISABLED";
        if (interactionMode !== "CREATE_GF") {
            if (viewer3d) viewer3d.updateGeofence([]);
        }
    }
}

// Map Click Handler
function onMapClick(x, y, z) {
    if (interactionMode === "ADD_WP") {
        ws.send(JSON.stringify({
            action: "add_waypoint",
            x: x, y: y, z: 0.0, // force z=0 for ground robot
            yaw: 0 // default, can be edited
        }));
    } else if (interactionMode === "SET_GOAL") {
        ws.send(JSON.stringify({
            action: "nav_goal",
            x: x, y: y, yaw: 0
        }));
        interactionMode = "NONE";
        document.getElementById('btn-set-goal').textContent = "SET SINGLE GOAL";
    } else if (interactionMode === "CREATE_GF") {
        tempPolygon.push({x: x, y: y});
        if (viewer3d) viewer3d.updateGeofence(tempPolygon);
    }
}

// Buttons
document.getElementById('btn-add-wp').addEventListener('click', () => {
    interactionMode = interactionMode === "ADD_WP" ? "NONE" : "ADD_WP";
    document.getElementById('btn-add-wp').textContent = interactionMode === "ADD_WP" ? "CLICK MAP TO ADD WP" : "ADD WAYPOINT";
});

document.getElementById('btn-set-goal').addEventListener('click', () => {
    if (mode !== "AUTO") {
        alert("Must be in AUTO mode to set goals");
        return;
    }
    interactionMode = interactionMode === "SET_GOAL" ? "NONE" : "SET_GOAL";
    document.getElementById('btn-set-goal').textContent = interactionMode === "SET_GOAL" ? "CLICK MAP FOR GOAL" : "SET SINGLE GOAL";
});

document.getElementById('btn-geofence').addEventListener('click', () => {
    interactionMode = "CREATE_GF";
    tempPolygon = [];
    document.getElementById('btn-geofence').style.display = "none";
    document.getElementById('btn-finish-gf').style.display = "inline-block";
});

document.getElementById('btn-finish-gf').addEventListener('click', () => {
    if (tempPolygon.length >= 3) {
        ws.send(JSON.stringify({
            action: "set_geofence",
            polygon: tempPolygon
        }));
    } else {
        alert("Geofence needs at least 3 points");
    }
    interactionMode = "NONE";
    document.getElementById('btn-geofence').style.display = "inline-block";
    document.getElementById('btn-finish-gf').style.display = "none";
});

document.getElementById('btn-clear-gf').addEventListener('click', () => {
    ws.send(JSON.stringify({action: "clear_geofence"}));
});

document.getElementById('btn-write-mission').addEventListener('click', () => {
    ws.send(JSON.stringify({action: "write_mission"}));
});

document.getElementById('btn-start-mission').addEventListener('click', () => {
    ws.send(JSON.stringify({action: "start_mission"}));
});

document.getElementById('btn-cancel-mission').addEventListener('click', () => {
    ws.send(JSON.stringify({action: "cancel_mission"}));
    document.getElementById('btn-start-mission').disabled = true;
});

document.getElementById('btn-clear-mission').addEventListener('click', () => {
    if(confirm("Clear entire mission?")) {
        ws.send(JSON.stringify({action: "clear_mission"}));
        document.getElementById('btn-start-mission').disabled = true;
    }
});

btnEstop.addEventListener('click', () => ws.send(JSON.stringify({action: "estop"})));
btnClearEstop.addEventListener('click', () => ws.send(JSON.stringify({action: "clear_estop"})));
btnManual.addEventListener('click', () => ws.send(JSON.stringify({action: "set_mode", mode: "MANUAL"})));
btnAuto.addEventListener('click', () => ws.send(JSON.stringify({action: "set_mode", mode: "AUTO"})));

// Waypoint Table
function renderWaypointTable() {
    wpList.innerHTML = '';
    waypoints.forEach((wp, index) => {
        const tr = document.createElement('tr');
        
        // ID
        let td = document.createElement('td');
        td.textContent = wp.id;
        tr.appendChild(td);
        
        // X
        td = document.createElement('td');
        td.innerHTML = `<input type="number" step="0.1" value="${wp.x.toFixed(2)}" onchange="updateWp(${wp.id}, 'x', this.value)">`;
        tr.appendChild(td);
        
        // Y
        td = document.createElement('td');
        td.innerHTML = `<input type="number" step="0.1" value="${wp.y.toFixed(2)}" onchange="updateWp(${wp.id}, 'y', this.value)">`;
        tr.appendChild(td);
        
        // Z
        td = document.createElement('td');
        td.innerHTML = `<input type="number" step="0.1" value="${wp.z.toFixed(2)}" onchange="updateWp(${wp.id}, 'z', this.value)">`;
        tr.appendChild(td);
        
        // YAW
        td = document.createElement('td');
        let yawDeg = (wp.yaw * 180 / Math.PI).toFixed(1);
        td.innerHTML = `<input type="number" step="1" value="${yawDeg}" onchange="updateWp(${wp.id}, 'yaw', this.value)">`;
        tr.appendChild(td);
        
        // Status
        td = document.createElement('td');
        td.textContent = wp.status;
        tr.appendChild(td);
        
        // Actions
        td = document.createElement('td');
        td.innerHTML = `
            <button onclick="reorderWp(${wp.id}, 'up')">↑</button>
            <button onclick="reorderWp(${wp.id}, 'down')">↓</button>
            <button onclick="deleteWp(${wp.id})" style="color:red;">✕</button>
        `;
        tr.appendChild(td);
        
        wpList.appendChild(tr);
    });
}

window.updateWp = function(id, field, val) {
    let wp = waypoints.find(w => w.id === id);
    if(!wp) return;
    
    let numVal = parseFloat(val);
    if(field === 'yaw') {
        numVal = numVal * Math.PI / 180.0;
    }
    
    ws.send(JSON.stringify({
        action: "update_waypoint",
        id: id,
        x: field === 'x' ? numVal : wp.x,
        y: field === 'y' ? numVal : wp.y,
        z: field === 'z' ? numVal : wp.z,
        yaw: field === 'yaw' ? numVal : wp.yaw
    }));
};

window.reorderWp = function(id, direction) {
    ws.send(JSON.stringify({action: "reorder_waypoint", id: id, direction: direction}));
};

window.deleteWp = function(id) {
    ws.send(JSON.stringify({action: "delete_waypoint", id: id}));
};


// Joystick Logic (same as before)
const joyZone = document.getElementById('joystick-zone');
const joyKnob = document.getElementById('joystick-knob');
let isDraggingJoy = false;
const maxLinearSpeed = 1.0;
const maxAngularSpeed = 1.5;

joyKnob.addEventListener('mousedown', () => { isDraggingJoy = true; });
document.addEventListener('mouseup', () => {
    if (isDraggingJoy) {
        isDraggingJoy = false;
        joyKnob.style.top = '50px';
        joyKnob.style.left = '50px';
        sendJoyCmd(0, 0);
    }
});
document.addEventListener('mousemove', (e) => {
    if (!isDraggingJoy) return;
    
    const rect = joyZone.getBoundingClientRect();
    const centerX = rect.left + 75;
    const centerY = rect.top + 75;
    
    let dx = e.clientX - centerX;
    let dy = e.clientY - centerY;
    
    const maxRadius = 50;
    const dist = Math.sqrt(dx*dx + dy*dy);
    
    if (dist > maxRadius) {
        dx = dx * maxRadius / dist;
        dy = dy * maxRadius / dist;
    }
    
    joyKnob.style.left = (50 + dx) + 'px';
    joyKnob.style.top = (50 + dy) + 'px';
    
    const normalizedY = -dy / maxRadius; // Forward is positive
    const normalizedX = -dx / maxRadius; // Left is positive angular.z
    
    const linear_x = normalizedY * maxLinearSpeed;
    const angular_z = normalizedX * maxAngularSpeed;
    
    sendJoyCmd(linear_x, angular_z);
});

let joyCmd = {x: 0, z: 0};
let joyTimer = null;

function sendJoyCmd(x, z) {
    joyCmd.x = x;
    joyCmd.z = z;
    
    if(Math.abs(x) < 0.1) joyCmd.x = 0;
    if(Math.abs(z) < 0.1) joyCmd.z = 0;
    
    if(joyCmd.x === 0 && joyCmd.z === 0) {
        if(joyTimer) { clearInterval(joyTimer); joyTimer = null; }
        ws.send(JSON.stringify({action: "joystick", linear_x: 0, angular_z: 0}));
    } else {
        if(!joyTimer) {
            joyTimer = setInterval(() => {
                ws.send(JSON.stringify({action: "joystick", linear_x: joyCmd.x, angular_z: joyCmd.z}));
            }, 100);
        }
    }
}
