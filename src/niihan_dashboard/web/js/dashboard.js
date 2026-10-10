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
let waypointSignature = null;
let waypointTableSignature = null;
let geofenceSignature = null;
let missionIsRunning = false;

// Interactions
let interactionMode = "NONE"; // NONE, ADD_WP, SET_GOAL, CREATE_GF
let tempPolygon = [];

// 3D Viewer
let viewer3d = null;
let viewer2d = null;

window.addEventListener('load', () => {
    viewer2d = new Viewer2D('map-canvas');
    viewer2d.setOnClickCallback(onMapClick);
    // An unavailable CDN or WebGL context must not disable the 2D planner.
    try {
        viewer3d = new Viewer3D('three-canvas-container');
        viewer3d.setOnClickCallback(onMapClick);
    } catch (error) {
        document.getElementById('map-status-3d').textContent = '3D VIEW UNAVAILABLE';
        console.warn('3D viewer unavailable:', error);
    }
    
    // Tab switching
    const tabBtns = document.querySelectorAll('.tab-btn');
    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.querySelectorAll('.view-container').forEach(c => c.classList.remove('active'));
            
            btn.classList.add('active');
            const target = document.getElementById(btn.dataset.target);
            target.classList.add('active');
            
            if (btn.dataset.target === 'view-3d' && viewer3d) {
                viewer3d.onWindowResize();
            } else if (btn.dataset.target === 'view-2d') {
                viewer2d.onWindowResize();
                viewer2d.requestRender();
            }
        });
    });
    if (!viewer3d) document.querySelector('[data-target="view-2d"]').click();
    document.getElementById('btn-fit-map').addEventListener('click', () => viewer2d.fitMap());
    connectWebSocket();
});

function showCommandStatus(message, success = false) {
    const element = document.getElementById('command-status');
    element.textContent = message;
    element.className = `progress-text ${success ? 'ok' : 'error'}`;
}

function sendCommand(command) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showCommandStatus('Dashboard disconnected; command was not sent.');
        return false;
    }
    ws.send(JSON.stringify(command));
    return true;
}

function updateGeofenceViews(polygon) {
    const signature = JSON.stringify(polygon);
    if (signature === geofenceSignature) return;
    geofenceSignature = signature;
    if (viewer3d) viewer3d.updateGeofence(polygon);
    if (viewer2d) viewer2d.updateGeofence(polygon);
}

function connectWebSocket() {
    ws = new WebSocket(`${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.hostname}:8081`);

    ws.onopen = () => {
        isConnected = true;
        elConn.textContent = "CONNECTED";
        elConn.className = "status-badge ok";
    };

    ws.onclose = () => {
        isConnected = false;
        camStream.removeAttribute("src");
        camStream.alt = "Camera disconnected";
        elConn.textContent = "DISCONNECTED";
        elConn.className = "status-badge error";
        document.getElementById('btn-start-mission').disabled = true;
        document.getElementById('btn-pause-mission').disabled = true;
        document.getElementById('btn-resume-mission').disabled = true;
        nav2Ready = false;
        elLoc.textContent = 'LOC: DISCONNECTED';elLoc.className = 'status-badge error';
        document.getElementById('slam-status').textContent = 'SLAM: DISCONNECTED';
        document.getElementById('tel-health').textContent = 'Supervisor disconnected';
        document.getElementById('vel-status').textContent = 'VEL: UNKNOWN';
        elNav.textContent = 'NAV2: UNKNOWN';
        elNav.className = 'status-badge error';
        document.getElementById('map-status-2d').textContent = '2D MAP: DISCONNECTED — DISPLAYING LAST MAP';
        if (joyTimer) { clearInterval(joyTimer); joyTimer = null; }
        isDraggingJoy = false;
        joyKnob.style.top = '50px';
        joyKnob.style.left = '50px';
        setTimeout(connectWebSocket, 2000);
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        if (data.type === "telemetry") {
            updateTelemetry(data);
        } else if (data.type === "map") {
            mapData = data;
            const valid = viewer2d && viewer2d.updateMap(mapData);
            document.getElementById('map-status-2d').textContent = valid ?
                `2D MAP: READY | ${data.width} × ${data.height} | ${data.resolution} m/cell | ${data.frame_id || 'map'}` :
                '2D MAP: INVALID GRID RECEIVED';
        } else if (data.type === "pointcloud") {
            if (viewer3d) {
                viewer3d.updatePointCloud(data.data);
                if (data.pose) viewer3d.updateRobotPose(data.pose.x,data.pose.y,data.pose.z,data.pose.yaw,data.pose.roll,data.pose.pitch);
                document.getElementById('map-status-3d').textContent = data.pose ?
                    `3D CLOUD ${data.stamp.toFixed(2)} s · X ${data.pose.x.toFixed(2)} Y ${data.pose.y.toFixed(2)} θ ${(data.pose.yaw*180/Math.PI).toFixed(1)}°` : '3D POINT CLOUD: READY';
            }
        } else if (data.type === "robot_model") {
            if (viewer3d) viewer3d.updateRobotModel(data);
        } else if (data.type === "joint_states") {
            if (viewer3d) viewer3d.updateJointStates(data.positions);
        } else if (data.type === "camera") {
            camStream.alt = "Forward camera live";
            camStream.src = "data:image/jpeg;base64," + data.image;
        } else if (data.type === "mission_write_response" || data.type === 'command_response' || data.type === 'error') {
            showCommandStatus(data.message, data.success === true);
        }
    };
}

function updateTelemetry(data) {
    const runtimeEl = document.getElementById('runtime-mode');
    if (runtimeEl) runtimeEl.textContent = (data.slam || {}).simulation === true ? 'SIMULATION' : 'UNKNOWN';
    const slamEl = document.getElementById('slam-status');
    if (slamEl) slamEl.textContent = 'SLAM: ' + ((data.slam || {}).backend || 'unknown');
    const health = data.hardware_health || {};
    const gnss = data.gnss || {};
    const healthEl = document.getElementById("tel-health");
    const gnssEl = document.getElementById("tel-gnss");
    if (healthEl) healthEl.textContent = health.ready ? "Sensors ready" : "Motion inhibited: " + ((health.faults || []).join(", ") || "waiting for supervisor");
    if (gnssEl) gnssEl.textContent = "GNSS: " + (gnss.quality || "unknown") + (Number.isFinite(gnss.latitude) && Number.isFinite(gnss.longitude) ? " | " + gnss.latitude.toFixed(7) + ", " + gnss.longitude.toFixed(7) : "");
    // Pose
    document.getElementById('tel-x').textContent = data.pose.x.toFixed(2);
    document.getElementById('tel-y').textContent = data.pose.y.toFixed(2);
    document.getElementById('tel-z').textContent = data.pose.z.toFixed(2);
    document.getElementById('tel-yaw').textContent = (data.pose.yaw * 180 / Math.PI).toFixed(1);
    
    document.getElementById('vel-status').textContent = 'VEL: ' + data.velocity.linear_x.toFixed(2) + ' m/s';
    // Vel
    document.getElementById('tel-vel-x').textContent = data.velocity.linear_x.toFixed(2);
    document.getElementById('tel-vel-z').textContent = data.velocity.angular_z.toFixed(2);
    
    // Status
    const poseValid = data.pose_valid !== false && (!data.localization.health || data.localization.health === 'OK');
    elLoc.textContent = `LOC: ${data.localization.source} ${data.localization.health || ''}`;
    elLoc.className = `status-badge ${poseValid ? 'ok' : 'error'}`;
    
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
        document.getElementById('btn-pause-mission').disabled = true;
        document.getElementById('btn-resume-mission').disabled = true;
    } else {
        btnEstop.style.display = "block";
        btnClearEstop.style.display = "none";
    }
    
    nav2Ready = data.nav2_ready;
    elNav.textContent = nav2Ready ? "NAV2: READY" : "NAV2: NOT AVAILABLE";
    elNav.className = nav2Ready ? "status-badge ok" : "status-badge error";
    
    robotPose = data.pose;
    if (viewer3d) viewer3d.updateRobotPose(robotPose.x, robotPose.y, robotPose.z, robotPose.yaw, robotPose.roll, robotPose.pitch);
    if (viewer2d) viewer2d.updateRobotPose(robotPose.x, robotPose.y, robotPose.yaw);
    
    // Mission
    const m = data.mission;
    missionState.textContent = m.state;
    missionIsRunning = ['RUNNING', 'CANCELLING', 'PAUSING', 'PAUSED'].includes(m.state);
    waypoints = m.waypoints;
    const signature = JSON.stringify(waypoints);
    if (signature !== waypointSignature) {
        waypointSignature = signature;
        if (viewer3d) viewer3d.updateWaypoints(waypoints);
        if (viewer2d) viewer2d.updateWaypoints(waypoints);
    }
    renderWaypointTable();
    const completed = waypoints.filter(waypoint => waypoint.status === 'COMPLETED').length;
    const active = waypoints.find(waypoint => waypoint.status === 'ACTIVE');
    missionProgress.textContent = `${completed}/${waypoints.length} reached${active ? ` · Active WP ${active.id}` : ''}`;
    const navigation = data.navigation || {state: 'UNKNOWN', message: ''};
    document.getElementById('navigation-status').textContent = `NAVIGATION: ${navigation.state}${navigation.message ? ` — ${navigation.message}` : ''}`;
    document.getElementById('btn-start-mission').disabled =
        !isConnected || m.state !== 'READY' || mode !== 'AUTO' || estopActive || !nav2Ready || !poseValid;
    
    document.getElementById('btn-pause-mission').disabled = !isConnected || m.state !== 'RUNNING';
    document.getElementById('btn-resume-mission').disabled = !isConnected || m.state !== 'PAUSED' || mode !== 'AUTO' || estopActive || !nav2Ready || !poseValid;
    // Geofence
    const gf = data.geofence;
    geofencePolygon = gf.enabled ? gf.polygon : [];
    gfState.textContent = interactionMode === 'CREATE_GF' ? `DRAFT (${tempPolygon.length} pts)` :
        (gf.enabled ? `ARMED (${gf.vertices} pts)` : 'DISABLED');
    updateGeofenceViews(interactionMode === 'CREATE_GF' ? tempPolygon : geofencePolygon);
}

// Map Click Handler
function onMapClick(x, y, z) {
    if (![x, y].every(Number.isFinite)) return;
    if (interactionMode === "ADD_WP") {
        sendCommand({
            action: "add_waypoint",
            x: x, y: y, z: robotPose.z
        });
    } else if (interactionMode === "SET_GOAL") {
        if (!sendCommand({
            action: "nav_goal",
            x: x, y: y, z: robotPose.z
        })) return;
        interactionMode = "NONE";
        document.getElementById('btn-set-goal').textContent = "SET SINGLE GOAL";
    } else if (interactionMode === "CREATE_GF") {
        tempPolygon.push({x: x, y: y});
        updateGeofenceViews(tempPolygon);
        gfState.textContent = `DRAFT (${tempPolygon.length} pts)`;
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
    updateGeofenceViews(tempPolygon);
    document.getElementById('btn-geofence').style.display = "none";
    document.getElementById('btn-finish-gf').style.display = "inline-block";
});

document.getElementById('btn-finish-gf').addEventListener('click', () => {
    if (tempPolygon.length >= 3) {
        if (!sendCommand({
            action: "set_geofence",
            polygon: tempPolygon
        })) return;
    } else {
        showCommandStatus('Geofence needs at least 3 points.');
        return;
    }
    interactionMode = "NONE";
    document.getElementById('btn-geofence').style.display = "inline-block";
    document.getElementById('btn-finish-gf').style.display = "none";
});

document.getElementById('btn-clear-gf').addEventListener('click', () => {
    sendCommand({action: "clear_geofence"});
});

document.getElementById('btn-write-mission').addEventListener('click', () => {
    sendCommand({action: "write_mission"});
});

document.getElementById('btn-start-mission').addEventListener('click', () => {
    sendCommand({action: "start_mission"});
});

document.getElementById('btn-pause-mission').addEventListener('click', () => sendCommand({action:'pause_mission'}));
document.getElementById('btn-resume-mission').addEventListener('click', () => sendCommand({action:'resume_mission'}));
document.getElementById('btn-cancel-mission').addEventListener('click', () => {
    sendCommand({action: "cancel_mission"});
    document.getElementById('btn-start-mission').disabled = true;
});

document.getElementById('btn-clear-mission').addEventListener('click', () => {
    if(confirm("Clear entire mission?")) {
        sendCommand({action: "clear_mission"});
        document.getElementById('btn-start-mission').disabled = true;
        document.getElementById('btn-pause-mission').disabled = true;
        document.getElementById('btn-resume-mission').disabled = true;
    }
});

btnEstop.addEventListener('click', () => sendCommand({action: "estop"}));
btnClearEstop.addEventListener('click', () => sendCommand({action: "clear_estop"}));
btnManual.addEventListener('click', () => sendCommand({action: "set_mode", mode: "MANUAL"}));
btnAuto.addEventListener('click', () => sendCommand({action: "set_mode", mode: "AUTO"}));

// Waypoint Table
function renderWaypointTable() {
    const signature = JSON.stringify([waypoints, missionIsRunning]);
    if (signature === waypointTableSignature) return;
    // Telemetry arrives several times a second. Preserve an in-progress edit.
    if (!missionIsRunning && wpList.contains(document.activeElement)) return;
    waypointTableSignature = signature;
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
    wpList.querySelectorAll('input, button').forEach(element => { element.disabled = missionIsRunning; });
}

window.updateWp = function(id, field, val) {
    let wp = waypoints.find(w => w.id === id);
    if(!wp) return;
    
    let numVal = parseFloat(val);
    if (!Number.isFinite(numVal)) {
        showCommandStatus('Waypoint coordinates and heading must be finite numbers.');
        waypointTableSignature = null;
        return;
    }
    if(field === 'yaw') {
        numVal = numVal * Math.PI / 180.0;
    }
    
    sendCommand({
        action: "update_waypoint",
        id: id,
        x: field === 'x' ? numVal : wp.x,
        y: field === 'y' ? numVal : wp.y,
        z: field === 'z' ? numVal : wp.z,
        yaw: field === 'yaw' ? numVal : wp.yaw
    });
};

window.reorderWp = function(id, direction) {
    sendCommand({action: "reorder_waypoint", id: id, direction: direction});
};

window.deleteWp = function(id) {
    sendCommand({action: "delete_waypoint", id: id});
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
        sendCommand({action: "joystick", linear_x: 0, angular_z: 0});
    } else {
        if(!joyTimer) {
            joyTimer = setInterval(() => {
                sendCommand({action: "joystick", linear_x: joyCmd.x, angular_z: joyCmd.z});
            }, 100);
        }
    }
}
