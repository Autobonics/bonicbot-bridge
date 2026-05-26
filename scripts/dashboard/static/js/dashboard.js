// ==========================================
// BonicBot Dashboard - Frontend Logic
// ==========================================

const socket = io();

// ── State ───────────────────────────────────────────────────────────────────

let mapState = {
    info:         null,      // last map_update payload (resolution, width, height, origin)
    imageData:    null,      // pre-rendered ImageData object (pixels, not raw array)
    lastTs:       0,         // ms timestamp of last map_update
    drawOffsetX:  0,         // pixel offset for centering on mapCanvas
    drawOffsetY:  0,
    scale:        1,         // pixels per cell
};

let poseState = {
    x:      0,               // metres
    y:      0,               // metres
    yaw:    0,               // radians
    lastTs: 0,               // ms timestamp of last robot_pose
};

const POSE_STALE_THRESHOLD_MS = 500;   // warn if pose is older than this

// Viewport state for Pan/Zoom
let panX = 0;
let panY = 0;
let isDragging = false;
let startDragX = 0;
let startDragY = 0;
let viewScale = 1.0; // renamed from scale to viewScale to avoid conflict with mapState.scale

// Canvas setup
const wrapper = document.getElementById('canvas-wrapper');
const mapCanvas = document.getElementById('map-canvas');
let mapCtx = null;
let poseCanvas = null;
let poseCtx = null;

document.addEventListener("DOMContentLoaded", () => {
    mapCtx = mapCanvas.getContext('2d');
    
    // Create pose canvas
    poseCanvas = document.createElement('canvas');
    poseCanvas.id = 'pose-canvas';
    poseCanvas.style.position = 'absolute';
    poseCanvas.style.top = '0';
    poseCanvas.style.left = '0';
    poseCanvas.style.width = '100%';
    poseCanvas.style.height = '100%';
    poseCanvas.style.pointerEvents = 'none';
    
    wrapper.style.position = 'relative';
    wrapper.appendChild(poseCanvas);
    
    poseCtx = poseCanvas.getContext('2d');
    
    resizeCanvases();
});

function resizeCanvases() {
    if (mapCanvas && wrapper) {
        if (mapCanvas.width !== wrapper.clientWidth || mapCanvas.height !== wrapper.clientHeight) {
            mapCanvas.width = wrapper.clientWidth;
            mapCanvas.height = wrapper.clientHeight;
        }
    }
    if (poseCanvas && wrapper) {
        if (poseCanvas.width !== wrapper.clientWidth || poseCanvas.height !== wrapper.clientHeight) {
            poseCanvas.width = wrapper.clientWidth;
            poseCanvas.height = wrapper.clientHeight;
        }
    }
}

window.addEventListener('resize', () => {
    resizeCanvases();
    renderMapLayer();
    renderPoseLayer();
});


// ── Render Helpers ──────────────────────────────────────────────────────────

function buildMapImageData(data, width, height) {
    if (!mapCtx) return null;
    const imgData = mapCtx.createImageData(width, height);
    
    for (let i = 0; i < data.length; i++) {
        const val = data[i];
        const idx = i * 4;
        
        if (val === -1) {
            imgData.data[idx] = 128;
            imgData.data[idx + 1] = 128;
            imgData.data[idx + 2] = 128;
            imgData.data[idx + 3] = 255;
        } else if (val === 0) {
            imgData.data[idx] = 255;
            imgData.data[idx + 1] = 255;
            imgData.data[idx + 2] = 255;
            imgData.data[idx + 3] = 255;
        } else {
            imgData.data[idx] = 0;
            imgData.data[idx + 1] = 0;
            imgData.data[idx + 2] = 0;
            imgData.data[idx + 3] = 255;
        }
    }
    return imgData;
}

function renderMapLayer() {
    if (!mapState.info || !mapState.imageData || !mapCtx) return;
    
    resizeCanvases();
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    
    const width = mapState.info.width;
    const height = mapState.info.height;
    const cW = mapCanvas.width;
    const cH = mapCanvas.height;
    
    const scale = Math.min(cW / width, cH / height);
    const drawOffsetX = (cW - width * scale) / 2;
    const drawOffsetY = (cH - height * scale) / 2;
    
    mapState.scale = scale;
    mapState.drawOffsetX = drawOffsetX;
    mapState.drawOffsetY = drawOffsetY;
    
    const off = new OffscreenCanvas(width, height);
    off.getContext('2d').putImageData(mapState.imageData, 0, 0);
    
    mapCtx.save();
    
    // Apply Pan & Zoom
    mapCtx.translate(cW / 2 + panX, cH / 2 + panY);
    mapCtx.scale(viewScale, viewScale);
    mapCtx.translate(-cW / 2, -cH / 2);
    
    mapCtx.translate(drawOffsetX + (width * scale) / 2, drawOffsetY + (height * scale) / 2);
    mapCtx.scale(scale, -scale);
    mapCtx.drawImage(off, -width / 2, -height / 2);
    
    mapCtx.restore();
}

function renderPoseLayer() {
    if (!mapState.info || !poseCtx) {
        if (poseCtx && poseCanvas) poseCtx.clearRect(0, 0, poseCanvas.width, poseCanvas.height);
        return;
    }
    
    resizeCanvases();
    poseCtx.clearRect(0, 0, poseCanvas.width, poseCanvas.height);
    
    const age = Date.now() - poseState.lastTs;
    const stale = age > POSE_STALE_THRESHOLD_MS;
    
    const originX = mapState.info.origin?.position?.x ?? 0;
    const originY = mapState.info.origin?.position?.y ?? 0;
    const res = mapState.info.resolution;
    
    const cx = (poseState.x - originX) / res;
    const cy = (poseState.y - originY) / res;
    
    poseCtx.save();
    
    const width = mapState.info.width;
    const height = mapState.info.height;
    const scale = mapState.scale;
    const drawOffsetX = mapState.drawOffsetX;
    const drawOffsetY = mapState.drawOffsetY;
    const cW = poseCanvas.width;
    const cH = poseCanvas.height;
    
    // Apply Pan & Zoom
    poseCtx.translate(cW / 2 + panX, cH / 2 + panY);
    poseCtx.scale(viewScale, viewScale);
    poseCtx.translate(-cW / 2, -cH / 2);
    
    poseCtx.translate(drawOffsetX + (width * scale) / 2, drawOffsetY + (height * scale) / 2);
    poseCtx.scale(scale, -scale);
    
    poseCtx.translate(cx - width / 2, cy - height / 2);
    poseCtx.rotate(-poseState.yaw);
    
    poseCtx.beginPath();
    poseCtx.moveTo(8, 0);
    poseCtx.lineTo(-5, 5);
    poseCtx.lineTo(-5, -5);
    poseCtx.closePath();
    
    poseCtx.fillStyle = stale ? '#e67e22' : '#e74c3c';
    poseCtx.fill();
    
    poseCtx.lineWidth = 1.5;
    poseCtx.strokeStyle = '#ffffff';
    poseCtx.stroke();
    
    poseCtx.restore();
}

// ── SocketIO Handlers ───────────────────────────────────────────────────────

socket.on('connect', () => {
    showToast('Connected to Dashboard server', 'success');
});

socket.on('disconnect', () => {
    showToast('Disconnected from server', 'error');
});

function updateHzCounter(ts) {
    if (mapState.lastTs > 0) {
        const hz = 1000 / (ts - mapState.lastTs);
        document.getElementById('fps-counter').textContent = `Map: ${hz.toFixed(1)} Hz`;
    }
    mapState.lastTs = ts;
}

socket.on('map_update', payload => {
    mapState.info = payload;
    const ts = payload.ts ?? Date.now();
    
    mapState.imageData = buildMapImageData(payload.data, payload.width, payload.height);
    
    renderMapLayer();
    renderPoseLayer();
    
    updateHzCounter(ts);
    
    const known = payload.data.filter(v => v !== -1).length;
    const area = known * (payload.resolution ** 2);
    document.getElementById('area-readout').textContent = `${area.toFixed(1)} m²`;
});

socket.on('map_tick', payload => {
    updateHzCounter(payload.ts ?? Date.now());
});

socket.on('robot_pose', payload => {
    poseState.x = payload.x;
    poseState.y = payload.y;
    poseState.yaw = payload.yaw;
    poseState.lastTs = payload.ts ?? Date.now();
    
    renderPoseLayer();
});

socket.on('robot_state', data => {
    const badge = document.getElementById('robot-state-badge');
    badge.innerText = `State: ${data.data}`;
    if(data.data === 'IDLE') {
        badge.className = 'badge badge-grey';
    } else if(data.data === 'ERROR') {
        badge.className = 'badge badge-red';
    } else {
        badge.className = 'badge badge-green';
    }
});

socket.on('mapping_active', data => {
    setIndicator('indicator-mapping', data.data);
    document.getElementById('btn-map-start').disabled = data.data;
    document.getElementById('btn-map-stop').disabled = !data.data;
});

socket.on('navigation_active', data => {
    setIndicator('indicator-navigation', data.data);
    document.getElementById('btn-nav-start').disabled = data.data;
    document.getElementById('btn-nav-stop').disabled = !data.data;
});

socket.on('nav_status', data => {
    document.getElementById('nav-status-text').textContent = data.data;
    document.getElementById('btn-cancel-goal').disabled = (data.data !== 'navigating');
});

socket.on('distance_to_goal', data => {
    document.getElementById('distance-readout').innerText = `${data.data.toFixed(2)} m`;
});

socket.on('locations_list', data => {
    const select = document.getElementById('select-location');
    const currentVal = select.value;
    
    select.innerHTML = '<option value="">Select location...</option>';
    if (data && data.data) {
        const locNames = Object.keys(data.data);
        locNames.forEach(name => {
            const opt = document.createElement('option');
            opt.value = name;
            opt.innerText = name;
            select.appendChild(opt);
        });
    }
    
    if (currentVal && Array.from(select.options).some(o => o.value === currentVal)) {
        select.value = currentVal;
    }
});


// ── Pan & Zoom Logic ────────────────────────────────────────────────────────

mapCanvas.addEventListener('wheel', e => {
    e.preventDefault();
    const zoomFactor = 1.1;
    if (e.deltaY < 0) {
        viewScale *= zoomFactor;
    } else {
        viewScale /= zoomFactor;
    }
    renderMapLayer();
    renderPoseLayer();
});

mapCanvas.addEventListener('mousedown', e => {
    isDragging = true;
    startDragX = e.clientX - panX;
    startDragY = e.clientY - panY;
});

window.addEventListener('mouseup', () => {
    isDragging = false;
});

window.addEventListener('mousemove', e => {
    if (!isDragging) return;
    panX = e.clientX - startDragX;
    panY = e.clientY - startDragY;
    renderMapLayer();
    renderPoseLayer();
});


// ── UI Helpers ──────────────────────────────────────────────────────────────

function setIndicator(id, active) {
    const el = document.getElementById(id);
    if (el) {
        if (active) {
            el.classList.add('indicator-active');
            el.classList.remove('indicator-inactive');
        } else {
            el.classList.add('indicator-inactive');
            el.classList.remove('indicator-active');
        }
    }
}

function showToast(message, type = 'info') {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerText = message;
    container.appendChild(toast);
    
    while (container.childElementCount > 5) {
        container.removeChild(container.firstChild);
    }
    
    setTimeout(() => {
        if (container.contains(toast)) {
            container.removeChild(toast);
        }
    }, 4000);
}

// ── REST API Calls ──────────────────────────────────────────────────────────

async function apiCall(endpoint, data = null) {
    try {
        const opts = { method: 'POST' };
        if (data) {
            opts.headers = { 'Content-Type': 'application/json' };
            opts.body = JSON.stringify(data);
        }
        const res = await fetch(endpoint, opts);
        const result = await res.json();
        
        if (result.success) {
            showToast(result.message, 'success');
            return { success: true, message: result.message };
        } else {
            showToast(result.message, 'error');
            return { success: false, message: result.message };
        }
    } catch (e) {
        showToast(`Request failed: ${e.message}`, 'error');
        return { success: false, message: e.message };
    }
}

function sendGoal() {
    const x = parseFloat(document.getElementById('input-x').value);
    const y = parseFloat(document.getElementById('input-y').value);
    const theta = parseFloat(document.getElementById('input-theta').value);
    
    if (isNaN(x) || isNaN(y) || isNaN(theta)) {
        showToast('Please enter valid numbers for coordinates', 'warning');
        return;
    }
    
    apiCall('/api/navigation/goal', {x: x, y: y, theta: theta});
}

function sendInitialPose() {
    const x = parseFloat(document.getElementById('input-x').value);
    const y = parseFloat(document.getElementById('input-y').value);
    const theta = parseFloat(document.getElementById('input-theta').value);
    
    if (isNaN(x) || isNaN(y) || isNaN(theta)) {
        showToast('Please enter valid numbers for coordinates', 'warning');
        return;
    }
    
    apiCall('/api/navigation/initial_pose', {x: x, y: y, theta: theta});
}

function cancelGoal() {
    apiCall('/api/navigation/cancel');
}

function gotoLocation() {
    const name = document.getElementById('select-location').value;
    if (!name) {
        showToast('Please select a location', 'warning');
        return;
    }
    apiCall('/api/navigation/goto', {name: name});
}

function saveLocation() {
    const name = document.getElementById('input-loc-name').value;
    if (!name) {
        showToast('Please enter a location name to save', 'warning');
        return;
    }
    apiCall('/api/navigation/save_location', {name: name});
}

function deleteLocation() {
    const name = document.getElementById('input-loc-name').value;
    if (!name) {
        showToast('Please enter a location name to delete', 'warning');
        return;
    }
    apiCall('/api/navigation/delete_location', {name: name});
}
