/**
 * ==========================================================================
 * AUV REMUS 100 MISSION PLANNER - APPLICATION ENGINE (V2.1)
 * High-Precision Ocean Trajectory Planning, Group Dragging & Heading Rotation
 * ==========================================================================
 */

// WGS-84 Ellipsoid Constants
const WGS84_A = 6378137.0;            // Semi-major axis (meters)
const WGS84_F = 1 / 298.257223563;    // Flattening
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared (~0.00669438)

// Preset Water Regions (100% Guaranteed On Open Water)
const REGION_PRESETS = {
  lanha: {
    name: 'Vịnh Lan Hạ - Cát Bà (Mặt biển mở)',
    lat: 20.720000,
    lon: 107.090000,
    zoom: 14
  },
  halong: {
    name: 'Vịnh Hạ Long (Vùng biển mở)',
    lat: 20.860000,
    lon: 107.130000,
    zoom: 14
  },
  hoabinh: {
    name: 'Hồ Hòa Bình (Lòng hồ đo sâu thực tế)',
    lat: 20.762200,
    lon: 105.062150,
    zoom: 14
  },
  camranh: {
    name: 'Vịnh Cam Ranh (Mặt nước)',
    lat: 11.890000,
    lon: 109.180000,
    zoom: 13
  },
  vungtau: {
    name: 'Vùng biển Vũng Tàu',
    lat: 10.330000,
    lon: 107.070000,
    zoom: 13
  }
};

// Global State
let map = null;
let currentBasemap = null;
let originLat = REGION_PRESETS.lanha.lat;
let originLon = REGION_PRESETS.lanha.lon;
let waypoints = []; // Array of Waypoint objects
let pathPolylines = [];
let segmentLabels = [];
let selectedWptId = null;
let addModeActive = true;
let pickOriginMode = false;
let currentHeadingDeg = 90; // Default running East
let selectedPattern = 'scurve';

/**
 * Coordinate Conversion: Lat/Lon -> Local NED (North, East in meters)
 */
function latLonToNed(lat, lon, lat0, lon0) {
  const latRad = (lat * Math.PI) / 180;
  const lat0Rad = (lat0 * Math.PI) / 180;
  const dLatRad = ((lat - lat0) * Math.PI) / 180;
  const dLonRad = ((lon - lon0) * Math.PI) / 180;

  const sinLat0 = Math.sin(lat0Rad);
  const RN = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat0 * sinLat0);
  const RM = (WGS84_A * (1 - WGS84_E2)) / Math.pow(1 - WGS84_E2 * sinLat0 * sinLat0, 1.5);

  const north = dLatRad * RM;
  const east = dLonRad * RN * Math.cos(lat0Rad);
  return { north, east };
}

/**
 * Coordinate Conversion: Local NED (North, East in meters) -> Lat/Lon
 */
function nedToLatLon(north, east, lat0, lon0) {
  const lat0Rad = (lat0 * Math.PI) / 180;
  const sinLat0 = Math.sin(lat0Rad);
  const RN = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat0 * sinLat0);
  const RM = (WGS84_A * (1 - WGS84_E2)) / Math.pow(1 - WGS84_E2 * sinLat0 * sinLat0, 1.5);

  const dLatRad = north / RM;
  const dLonRad = east / (RN * Math.cos(lat0Rad));

  const lat = lat0 + (dLatRad * 180) / Math.PI;
  const lon = lon0 + (dLonRad * 180) / Math.PI;
  return { lat, lon };
}

/**
 * Initialize Map and Layers
 */
function initMap() {
  const preset = REGION_PRESETS.lanha;
  originLat = preset.lat;
  originLon = preset.lon;

  map = L.map('map', {
    center: [preset.lat, preset.lon],
    zoom: preset.zoom,
    maxZoom: 21,
    zoomControl: false
  });

  // Zoom control in bottom right
  L.control.zoom({ position: 'bottomright' }).addTo(map);

  // Precision Metric Scale Bar in bottom left
  L.control.scale({ metric: true, imperial: false, position: 'bottomleft', maxWidth: 150 }).addTo(map);

  // Basemap definitions
  const basemaps = {
    'googleHybrid': L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
      maxZoom: 21,
      maxNativeZoom: 19,
      attribution: '© Google Maps Satellite'
    }),
    'googleSat': L.tileLayer('https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
      maxZoom: 21,
      maxNativeZoom: 19,
      attribution: '© Google Satellite'
    }),
    'esriSat': L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 21,
      maxNativeZoom: 19,
      attribution: '© Esri World Imagery'
    }),
    'cartoDark': L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      maxZoom: 21,
      maxNativeZoom: 19,
      attribution: '© CartoDB Dark'
    })
  };

  currentBasemap = basemaps.googleHybrid;
  currentBasemap.addTo(map);

  // Map Basemap Selector
  document.getElementById('basemapSelect').addEventListener('change', (e) => {
    map.removeLayer(currentBasemap);
    currentBasemap = basemaps[e.target.value] || basemaps.googleHybrid;
    currentBasemap.addTo(map);
  });

  // Preset Selector Event
  document.getElementById('regionSelect').addEventListener('change', (e) => {
    const p = REGION_PRESETS[e.target.value];
    if (p) {
      originLat = p.lat;
      originLon = p.lon;
      map.flyTo([p.lat, p.lon], p.zoom, { duration: 1.2 });
      if (waypoints.length > 0) {
        // Shift entire trajectory to new region water center
        shiftEntireTrajectoryTo(p.lat, p.lon);
      }
    }
  });

  // Map Cursor Tracker
  map.on('mousemove', (e) => {
    const lat = e.latlng.lat;
    const lon = e.latlng.lng;
    const ned = latLonToNed(lat, lon, originLat, originLon);
    document.getElementById('cursorLat').innerText = lat.toFixed(6);
    document.getElementById('cursorLon').innerText = lon.toFixed(6);
    document.getElementById('cursorNorth').innerText = `${ned.north >= 0 ? '+' : ''}${ned.north.toFixed(1)} m`;
    document.getElementById('cursorEast').innerText = `${ned.east >= 0 ? '+' : ''}${ned.east.toFixed(1)} m`;
  });

  // Adaptive Tactical Marker Scaling on High Zoom
  map.on('zoomend', () => {
    const container = map.getContainer();
    if (map.getZoom() >= 17) {
      container.classList.add('zoom-micro');
    } else {
      container.classList.remove('zoom-micro');
    }
  });

  // Map Click Handler (Supports Add Waypoint and Pick Origin Mode)
  map.on('click', (e) => {
    if (pickOriginMode) {
      // Teleport entire trajectory to clicked water point
      shiftEntireTrajectoryTo(e.latlng.lat, e.latlng.lng);
      stopPickOriginMode();
      return;
    }

    if (!addModeActive) return;
    addWaypoint(e.latlng.lat, e.latlng.lng);
  });

  // Load Default 12-Waypoint Remus 100 Scenario placed 100% on open water
  loadDefaultScenario();
}

/**
 * Add a New Waypoint
 */
function addWaypoint(lat, lon, depth = null, speed = 1.5) {
  const id = waypoints.length + 1;
  
  if (waypoints.length === 0) {
    originLat = lat;
    originLon = lon;
  }

  const ned = latLonToNed(lat, lon, originLat, originLon);

  let targetDepth = depth;
  if (targetDepth === null) {
    if (waypoints.length < 6) {
      targetDepth = 0.0;
    } else if (waypoints.length === 6) {
      targetDepth = 10.0;
    } else {
      targetDepth = waypoints[waypoints.length - 1].depth || 50.0;
    }
  }

  const wpt = {
    id: id,
    lat: lat,
    lon: lon,
    north: ned.north,
    east: ned.east,
    depth: targetDepth,
    speed: speed,
    marker: null
  };

  createMarkerForWpt(wpt);
  waypoints.push(wpt);

  updateUI();
  selectWaypoint(wpt.id);

  setTimeout(() => {
    const container = document.querySelector('.wpt-table-container');
    if (container) {
      container.scrollTop = container.scrollHeight;
    }
    const newRow = document.getElementById(`wpt-row-${wpt.id}`);
    if (newRow) {
      newRow.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      const depthInput = newRow.querySelector('input[title*="Độ sâu"]');
      if (depthInput) {
        depthInput.focus();
        depthInput.select();
      }
    }
  }, 40);
}

/**
 * Create Leaflet Marker with GROUP DRAG SUPPORT on WP1
 */
function createMarkerForWpt(wpt) {
  if (wpt.marker) {
    map.removeLayer(wpt.marker);
  }

  const isSurface = wpt.depth <= 0.5;
  const markerClass = isSurface ? 'marker-surface' : 'marker-dive';

  const icon = L.divIcon({
    className: '',
    html: `<div class="custom-wpt-marker ${markerClass}" id="marker-${wpt.id}">WP${wpt.id}</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  });

  const marker = L.marker([wpt.lat, wpt.lon], {
    icon: icon,
    draggable: true
  }).addTo(map);

  let startDragLatLng = null;

  marker.on('dragstart', (e) => {
    startDragLatLng = marker.getLatLng();
  });

  marker.on('drag', (e) => {
    const pos = marker.getLatLng();
    const groupMoveEnabled = document.getElementById('groupMoveToggle')?.checked;

    if (wpt.id === 1 && groupMoveEnabled && startDragLatLng) {
      // Group drag: shift ALL subsequent waypoints along with WP1!
      const dLat = pos.lat - startDragLatLng.lat;
      const dLon = pos.lng - startDragLatLng.lng;

      wpt.lat = pos.lat;
      wpt.lon = pos.lng;
      originLat = wpt.lat;
      originLon = wpt.lon;

      for (let i = 1; i < waypoints.length; i++) {
        const w = waypoints[i];
        w.lat += dLat;
        w.lon += dLon;
        if (w.marker) {
          w.marker.setLatLng([w.lat, w.lon]);
        }
      }

      startDragLatLng = pos;
      recalculateAllNed(false);
      renderPolylines();
      renderDepthProfile();
      updateTableRowsOnly();
      updateMetrics();
      return;
    }

    // Single waypoint drag
    wpt.lat = pos.lat;
    wpt.lon = pos.lng;
    const ned = latLonToNed(wpt.lat, wpt.lon, originLat, originLon);
    wpt.north = ned.north;
    wpt.east = ned.east;

    renderPolylines();
    renderDepthProfile();
    updateTableRowsOnly();
    updateMetrics();
  });

  marker.on('dragend', () => {
    startDragLatLng = null;
    updateUI();
  });

  marker.on('click', () => {
    selectWaypoint(wpt.id);
  });

  wpt.marker = marker;
}

/**
 * Shift ENTIRE Trajectory to a New Water Location (Lat, Lon)
 */
function shiftEntireTrajectoryTo(newLat, newLon) {
  if (waypoints.length === 0) {
    originLat = newLat;
    originLon = newLon;
    return;
  }

  const dLat = newLat - waypoints[0].lat;
  const dLon = newLon - waypoints[0].lon;

  originLat = newLat;
  originLon = newLon;

  waypoints.forEach(w => {
    w.lat += dLat;
    w.lon += dLon;
    if (w.marker) {
      w.marker.setLatLng([w.lat, w.lon]);
    }
  });

  recalculateAllNed(false);
  updateUI();
  map.panTo([newLat, newLon], { animate: true });
}

/**
 * Pick Origin on Water Mode (Click any water location to relocate entire mission)
 */
function startPickOriginMode() {
  pickOriginMode = true;
  document.getElementById('mapStatusText').innerHTML = '<span style="color:#f59e0b; font-weight:700;">📍 HÃY CLICK TRÊN MẶT NƯỚC: Toàn bộ quỹ đạo sẽ lập tức chuyển đến đó!</span>';
  document.getElementById('mapStatusBar').style.borderColor = '#f59e0b';
  document.getElementById('btnPickOrigin').classList.add('btn-amber');
  document.getElementById('btnPickOrigin').innerText = '✕ Hủy Chọn Biển';
}

function stopPickOriginMode() {
  pickOriginMode = false;
  document.getElementById('mapStatusText').innerHTML = 'Chế độ: <b>Kéo WP1 để dời quỹ đạo hoặc click trên biển để thêm điểm</b>';
  document.getElementById('mapStatusBar').style.borderColor = 'var(--border-color)';
  document.getElementById('btnPickOrigin').classList.remove('btn-amber');
  document.getElementById('btnPickOrigin').innerText = '📍 Click Biển Đặt Lại Gốc';
}

/**
 * Rotate Trajectory Heading Around WP1 (0° - 360°)
 */
function onRotateSlider(newAngle) {
  const targetAngle = parseFloat(newAngle) || 90;
  const deltaDeg = targetAngle - currentHeadingDeg;
  if (Math.abs(deltaDeg) < 0.01) return;

  const deltaRad = (deltaDeg * Math.PI) / 180;
  const cosA = Math.cos(deltaRad);
  const sinA = Math.sin(deltaRad);

  // Rotate all waypoints relative to WP1 (which is at origin 0,0)
  for (let i = 1; i < waypoints.length; i++) {
    const w = waypoints[i];
    const nNew = w.north * cosA - w.east * sinA;
    const eNew = w.north * sinA + w.east * cosA;
    w.north = nNew;
    w.east = eNew;

    const geo = nedToLatLon(w.north, w.east, originLat, originLon);
    w.lat = geo.lat;
    w.lon = geo.lon;
    if (w.marker) {
      w.marker.setLatLng([w.lat, w.lon]);
    }
  }

  currentHeadingDeg = targetAngle;

  // Update Display Heading text
  const compassName = getCompassDirection(currentHeadingDeg);
  document.getElementById('headingDisplay').innerText = `${Math.round(currentHeadingDeg).toString().padStart(3, '0')}° (${compassName})`;

  renderPolylines();
  renderDepthProfile();
  updateTableRowsOnly();
  updateMetrics();
}

function getCompassDirection(deg) {
  const d = (deg % 360 + 360) % 360;
  if (d >= 337.5 || d < 22.5) return 'Bắc';
  if (d >= 22.5 && d < 67.5) return 'Đông Bắc';
  if (d >= 67.5 && d < 112.5) return 'Đông';
  if (d >= 112.5 && d < 157.5) return 'Đông Nam';
  if (d >= 157.5 && d < 202.5) return 'Nam';
  if (d >= 202.5 && d < 247.5) return 'Tây Nam';
  if (d >= 247.5 && d < 292.5) return 'Tây';
  return 'Tây Bắc';
}

/**
 * Re-render Map Polylines with Color-Coding
 */
function renderPolylines() {
  pathPolylines.forEach(p => map.removeLayer(p));
  pathPolylines = [];
  segmentLabels.forEach(l => map.removeLayer(l));
  segmentLabels = [];

  if (waypoints.length < 2) return;

  for (let i = 0; i < waypoints.length - 1; i++) {
    const p1 = waypoints[i];
    const p2 = waypoints[i + 1];

    const isSurfaceSegment = (p1.depth <= 0.5) && (p2.depth <= 0.5);
    const color = isSurfaceSegment ? '#10b981' : '#00f0ff';
    const dashArray = isSurfaceSegment ? null : '6, 6';

    const polyline = L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], {
      color: color,
      weight: 3.5,
      opacity: 0.9,
      dashArray: dashArray
    }).addTo(map);
    pathPolylines.push(polyline);

    // Midpoint distance and heading label
    const midLat = (p1.lat + p2.lat) / 2;
    const midLon = (p1.lon + p2.lon) / 2;
    const dx = p2.north - p1.north;
    const dy = p2.east - p1.east;
    const dz = p2.depth - p1.depth;
    const dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const headingDeg = (Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360;

    const labelIcon = L.divIcon({
      className: '',
      html: `<div style="background: rgba(10,17,32,0.85); color: #94a3b8; border: 1px solid rgba(56,189,248,0.25); padding: 1px 5px; border-radius: 4px; font-size: 8.5px; font-family: monospace; white-space: nowrap; transform: translate(-50%, -50%);">
              <span style="color:#00f0ff; font-weight:700;">${dist3D.toFixed(0)}m</span> | ${headingDeg.toFixed(0)}°
             </div>`,
      iconSize: [0, 0]
    });
    const labelMarker = L.marker([midLat, midLon], { icon: labelIcon, interactive: false }).addTo(map);
    segmentLabels.push(labelMarker);
  }
}

/**
 * Render 2D Depth Profile (Cross-Section) on Canvas
 */
function renderDepthProfile() {
  const canvas = document.getElementById('profileCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  
  const width = canvas.parentElement.clientWidth;
  const height = canvas.parentElement.clientHeight;
  canvas.width = width * window.devicePixelRatio;
  canvas.height = height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

  ctx.clearRect(0, 0, width, height);

  if (waypoints.length === 0) {
    ctx.fillStyle = '#64748b';
    ctx.font = '11px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('Chưa có Waypoint nào. Click trên bản đồ để thêm mốc.', width / 2, height / 2);
    return;
  }

  const dists = [0];
  let maxDepth = 10;
  for (let i = 1; i < waypoints.length; i++) {
    const p1 = waypoints[i - 1];
    const p2 = waypoints[i];
    const dx = p2.north - p1.north;
    const dy = p2.east - p1.east;
    const dz = p2.depth - p1.depth;
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    dists.push(dists[i - 1] + d);
    if (p2.depth > maxDepth) maxDepth = p2.depth;
  }
  if (waypoints[0].depth > maxDepth) maxDepth = waypoints[0].depth;
  maxDepth = Math.max(maxDepth, 30);

  const totalDist = dists[dists.length - 1];
  const padding = { left: 45, right: 30, top: 18, bottom: 22 };
  const graphWidth = width - padding.left - padding.right;
  const graphHeight = height - padding.top - padding.bottom;

  const surfaceY = padding.top;

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#64748b';
  ctx.font = '9px JetBrains Mono';
  ctx.textAlign = 'right';

  const depthSteps = [0, 20, 50, 100, 150];
  depthSteps.forEach(d => {
    if (d <= maxDepth * 1.15) {
      const y = padding.top + (d / (maxDepth * 1.15)) * graphHeight;
      ctx.beginPath();
      ctx.moveTo(padding.left, y);
      ctx.lineTo(width - padding.right, y);
      ctx.stroke();
      ctx.fillText(`${d}m`, padding.left - 6, y + 3);
    }
  });

  ctx.fillStyle = '#10b981';
  ctx.fillText('Mặt nước (z=0)', padding.left + 85, surfaceY - 4);

  if (totalDist <= 0) return;

  const coords = waypoints.map((wpt, idx) => {
    const x = padding.left + (dists[idx] / totalDist) * graphWidth;
    const y = padding.top + (wpt.depth / (maxDepth * 1.15)) * graphHeight;
    return { x, y, wpt };
  });

  const gradient = ctx.createLinearGradient(0, surfaceY, 0, height);
  gradient.addColorStop(0, 'rgba(16, 185, 129, 0.15)');
  gradient.addColorStop(0.3, 'rgba(6, 182, 212, 0.25)');
  gradient.addColorStop(1, 'rgba(2, 132, 199, 0.45)');

  ctx.beginPath();
  ctx.moveTo(coords[0].x, coords[0].y);
  for (let i = 1; i < coords.length; i++) {
    ctx.lineTo(coords[i].x, coords[i].y);
  }
  ctx.lineTo(coords[coords.length - 1].x, height - padding.bottom);
  ctx.lineTo(coords[0].x, height - padding.bottom);
  ctx.closePath();
  ctx.fillStyle = gradient;
  ctx.fill();

  for (let i = 0; i < coords.length - 1; i++) {
    const c1 = coords[i];
    const c2 = coords[i + 1];
    const isSurface = (c1.wpt.depth <= 0.5) && (c2.wpt.depth <= 0.5);

    ctx.beginPath();
    ctx.moveTo(c1.x, c1.y);
    ctx.lineTo(c2.x, c2.y);
    ctx.strokeStyle = isSurface ? '#10b981' : '#00f0ff';
    ctx.lineWidth = 2.5;
    ctx.stroke();
  }

  coords.forEach((c, idx) => {
    const isSurface = c.wpt.depth <= 0.5;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = isSurface ? '#10b981' : '#00f0ff';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    ctx.fillStyle = '#f8fafc';
    ctx.font = '9px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(`WP${idx + 1} (${c.wpt.depth}m)`, c.x, c.y - 8);
  });
}

/**
 * Update Sidebar Waypoint Table with DIRECT EDITABLE INPUTS for North & East
 */
function updateTable() {
  const tbody = document.getElementById('wptTableBody');
  tbody.innerHTML = '';

  waypoints.forEach((wpt, idx) => {
    const tr = document.createElement('tr');
    tr.id = `wpt-row-${wpt.id}`;
    if (selectedWptId === wpt.id) tr.classList.add('active');

    const isSurface = wpt.depth <= 0.5;
    const tagHtml = isSurface 
      ? '<span class="wpt-tag wpt-tag-surface">Nổi GNSS</span>'
      : '<span class="wpt-tag wpt-tag-dive">Lặn DVL</span>';

    tr.innerHTML = `
      <td style="font-weight:700; color:var(--accent-cyan); white-space:nowrap;">WP${wpt.id}</td>
      <td>
        <input type="number" step="5" class="wpt-input" style="width:58px;" value="${wpt.north.toFixed(1)}"
          onchange="updateWptNorth(${wpt.id}, this.value)" title="Tọa độ North x (m)">
      </td>
      <td>
        <input type="number" step="5" class="wpt-input" style="width:58px;" value="${wpt.east.toFixed(1)}"
          onchange="updateWptEast(${wpt.id}, this.value)" title="Tọa độ East y (m)">
      </td>
      <td>
        <input type="number" step="1" min="0" max="200" class="wpt-input" style="width:44px;" value="${wpt.depth}"
          onchange="updateWptDepth(${wpt.id}, this.value)" title="Độ sâu z (m)">
      </td>
      <td>
        <input type="number" step="0.1" min="0.5" max="3.0" class="wpt-input" style="width:40px;" value="${wpt.speed}"
          onchange="updateWptSpeed(${wpt.id}, this.value)" title="Vận tốc (m/s)">
      </td>
      <td>${tagHtml}</td>
      <td style="text-align:center;">
        <button class="btn btn-danger btn-sm" style="padding:1px 6px; font-size:11px;" onclick="deleteWaypoint(${wpt.id}, event)" title="Xóa waypoint">×</button>
      </td>
    `;

    tr.addEventListener('click', (e) => {
      if (e.target.tagName !== 'INPUT' && e.target.tagName !== 'BUTTON') {
        selectWaypoint(wpt.id);
      }
    });

    tbody.appendChild(tr);
  });
}

function updateTableRowsOnly() {
  waypoints.forEach(wpt => {
    const row = document.getElementById(`wpt-row-${wpt.id}`);
    if (row) {
      const inputs = row.querySelectorAll('input');
      if (inputs.length >= 2) {
        if (document.activeElement !== inputs[0]) inputs[0].value = wpt.north.toFixed(1);
        if (document.activeElement !== inputs[1]) inputs[1].value = wpt.east.toFixed(1);
      }
    }
  });
}

/**
 * Update Direct North / East Inputs
 */
function updateWptNorth(id, val) {
  const wpt = waypoints.find(w => w.id === id);
  if (wpt) {
    wpt.north = parseFloat(val) || 0.0;
    const geo = nedToLatLon(wpt.north, wpt.east, originLat, originLon);
    wpt.lat = geo.lat;
    wpt.lon = geo.lon;
    if (wpt.marker) {
      wpt.marker.setLatLng([wpt.lat, wpt.lon]);
    }
    renderPolylines();
    renderDepthProfile();
    updateMetrics();
  }
}

function updateWptEast(id, val) {
  const wpt = waypoints.find(w => w.id === id);
  if (wpt) {
    wpt.east = parseFloat(val) || 0.0;
    const geo = nedToLatLon(wpt.north, wpt.east, originLat, originLon);
    wpt.lat = geo.lat;
    wpt.lon = geo.lon;
    if (wpt.marker) {
      wpt.marker.setLatLng([wpt.lat, wpt.lon]);
    }
    renderPolylines();
    renderDepthProfile();
    updateMetrics();
  }
}

/**
 * Update Mission Telemetry Metrics (Clean, No-Fluff)
 */
function updateMetrics() {
  let totalDist = 0;
  let totalTime = 0;
  let maxDepth = 0;

  for (let i = 0; i < waypoints.length; i++) {
    const p = waypoints[i];
    if (p.depth > maxDepth) maxDepth = p.depth;

    if (i > 0) {
      const pPrev = waypoints[i - 1];
      const dx = p.north - pPrev.north;
      const dy = p.east - pPrev.east;
      const dz = p.depth - pPrev.depth;
      const segDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const avgSpeed = (p.speed + pPrev.speed) / 2 || 1.5;
      const segTime = segDist / avgSpeed;

      totalDist += segDist;
      totalTime += segTime;
    }
  }

  document.getElementById('metricWptCount').innerText = waypoints.length;
  document.getElementById('metricDistance').innerHTML = `${(totalDist / 1000).toFixed(2)} <span class="metric-unit">km</span>`;
  
  const minutes = Math.floor(totalTime / 60);
  const seconds = Math.floor(totalTime % 60);
  document.getElementById('metricTime').innerHTML = `${minutes}m ${seconds}s <span class="metric-unit">(${totalTime.toFixed(0)}s)</span>`;
  document.getElementById('metricMaxDepth').innerHTML = `${maxDepth.toFixed(0)} <span class="metric-unit">m</span>`;
}

/**
 * Select / Highlight Waypoint
 */
function selectWaypoint(id) {
  selectedWptId = id;
  const wpt = waypoints.find(w => w.id === id);
  if (!wpt) return;

  map.panTo([wpt.lat, wpt.lon], { animate: true });

  document.querySelectorAll('#wptTableBody tr').forEach(r => r.classList.remove('active'));
  const row = document.getElementById(`wpt-row-${id}`);
  if (row) {
    row.classList.add('active');
    row.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  }
}

/**
 * Waypoint Edit Handlers
 */
function updateWptDepth(id, val) {
  const wpt = waypoints.find(w => w.id === id);
  if (wpt) {
    wpt.depth = parseFloat(val) || 0.0;
    createMarkerForWpt(wpt);
    updateUI();
  }
}

function updateWptSpeed(id, val) {
  const wpt = waypoints.find(w => w.id === id);
  if (wpt) {
    wpt.speed = parseFloat(val) || 1.5;
    updateMetrics();
  }
}

function deleteWaypoint(id, e) {
  if (e) e.stopPropagation();
  const idx = waypoints.findIndex(w => w.id === id);
  if (idx !== -1) {
    map.removeLayer(waypoints[idx].marker);
    waypoints.splice(idx, 1);
    
    waypoints.forEach((w, i) => {
      w.id = i + 1;
      createMarkerForWpt(w);
    });

    updateUI();
  }
}

function clearAllWaypoints() {
  if (confirm('Bạn có chắc muốn xóa tất cả các Waypoint?')) {
    waypoints.forEach(w => map.removeLayer(w.marker));
    waypoints = [];
    selectedWptId = null;
    updateUI();
  }
}

function recalculateAllNed(updateOrigin = true) {
  if (waypoints.length === 0) return;
  if (updateOrigin) {
    originLat = waypoints[0].lat;
    originLon = waypoints[0].lon;
  }

  waypoints.forEach(w => {
    const ned = latLonToNed(w.lat, w.lon, originLat, originLon);
    w.north = ned.north;
    w.east = ned.east;
  });
}

/**
 * Zoom Cameras
 */
function zoomToCalibrationZone() {
  const calibWpts = waypoints.filter(w => w.depth <= 0.5);
  if (calibWpts.length > 0) {
    const bounds = L.latLngBounds(calibWpts.map(w => [w.lat, w.lon]));
    map.fitBounds(bounds, { padding: [80, 80], maxZoom: 19 });
  } else {
    alert('Không tìm thấy mốc hiệu chuẩn mặt nước (z <= 0.5m) nào để zoom!');
  }
}

function zoomToFullMission() {
  if (waypoints.length > 0) {
    const bounds = L.latLngBounds(waypoints.map(w => [w.lat, w.lon]));
    map.fitBounds(bounds, { padding: [50, 50] });
  }
}

/**
 * CALIBRATION PATTERN GENERATOR ENGINE
 */
function openCalibrationModal() {
  document.getElementById('calibModal').classList.add('open');
}

function closeCalibrationModal() {
  document.getElementById('calibModal').classList.remove('open');
}

function selectCalibPattern(type) {
  selectedPattern = type;
  document.querySelectorAll('.pattern-card').forEach(c => c.classList.remove('active'));
  const card = document.getElementById(`pattern-${type}`);
  if (card) card.classList.add('active');

  if (type === 'scurve') {
    document.getElementById('calibStepDist').value = 150;
    document.getElementById('calibAmplitude').value = 30;
    document.getElementById('calibCycles').value = 3;
  } else if (type === 'zigzag') {
    document.getElementById('calibStepDist').value = 120;
    document.getElementById('calibAmplitude').value = 40;
    document.getElementById('calibCycles').value = 3;
  } else if (type === 'figure8') {
    document.getElementById('calibStepDist').value = 100;
    document.getElementById('calibAmplitude').value = 50;
    document.getElementById('calibCycles').value = 2;
  } else if (type === 'speedstep') {
    document.getElementById('calibStepDist').value = 200;
    document.getElementById('calibAmplitude').value = 10;
    document.getElementById('calibCycles').value = 3;
  }
}

/**
 * Apply Generated Calibration Pattern to Mission
 */
function applyCalibrationPattern() {
  const stepDist = parseFloat(document.getElementById('calibStepDist').value) || 150;
  const ampl = parseFloat(document.getElementById('calibAmplitude').value) || 30;
  const cycles = parseInt(document.getElementById('calibCycles').value) || 3;
  const speed = parseFloat(document.getElementById('calibSpeed').value) || 1.5;

  const existingDiveWpts = waypoints.filter(w => w.depth > 0.5);

  const calibNeds = [];
  
  if (selectedPattern === 'scurve') {
    calibNeds.push({ n: 0, e: 0, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.8, z: 0, u: speed });
    let curE = stepDist * 0.8;
    for (let c = 0; c < cycles; c++) {
      curE += stepDist;
      const sign = (c % 2 === 0) ? -1 : 1;
      calibNeds.push({ n: sign * ampl, e: curE, z: 0, u: speed });
    }
    curE += stepDist;
    calibNeds.push({ n: 0, e: curE, z: 0, u: speed });

  } else if (selectedPattern === 'zigzag') {
    calibNeds.push({ n: 0, e: 0, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist, z: 0, u: speed });
    let curE = stepDist;
    for (let c = 0; c < cycles; c++) {
      const sign = (c % 2 === 0) ? -1 : 1;
      calibNeds.push({ n: sign * ampl, e: curE + stepDist * 0.5, z: 0, u: speed });
      calibNeds.push({ n: sign * ampl, e: curE + stepDist, z: 0, u: speed });
      curE += stepDist;
    }
    calibNeds.push({ n: 0, e: curE + stepDist * 0.5, z: 0, u: speed });

  } else if (selectedPattern === 'figure8') {
    const R = ampl;
    calibNeds.push({ n: 0, e: 0, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5, z: 0, u: speed });
    calibNeds.push({ n: R, e: stepDist * 0.5 + R, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5 + 2 * R, z: 0, u: speed });
    calibNeds.push({ n: -R, e: stepDist * 0.5 + R, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5, z: 0, u: speed });
    calibNeds.push({ n: -R, e: stepDist * 0.5 - R, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5 - 2 * R, z: 0, u: speed });
    calibNeds.push({ n: R, e: stepDist * 0.5 - R, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5, z: 0, u: speed });
    calibNeds.push({ n: 0, e: stepDist * 0.5 + 2 * R + 100, z: 0, u: speed });

  } else if (selectedPattern === 'speedstep') {
    calibNeds.push({ n: 0, e: 0, z: 0, u: 1.0 });
    calibNeds.push({ n: 0, e: stepDist, z: 0, u: 1.0 });
    calibNeds.push({ n: 0, e: stepDist * 2, z: 0, u: 2.0 });
    calibNeds.push({ n: 0, e: stepDist * 3, z: 0, u: 2.0 });
    calibNeds.push({ n: 0, e: stepDist * 4, z: 0, u: 1.5 });
  }

  // Rotate newly generated calib pattern to match currentHeadingDeg (relative to East = 90)
  const rotRad = ((currentHeadingDeg - 90) * Math.PI) / 180;
  const cosR = Math.cos(rotRad);
  const sinR = Math.sin(rotRad);

  calibNeds.forEach(item => {
    const nR = item.n * cosR - item.e * sinR;
    const eR = item.n * sinR + item.e * cosR;
    item.n = nR;
    item.e = eR;
  });

  // Clear existing markers
  waypoints.forEach(w => map.removeLayer(w.marker));
  waypoints = [];

  // Add Phase 1 Calibration Waypoints
  calibNeds.forEach(item => {
    const geo = nedToLatLon(item.n, item.e, originLat, originLon);
    addWaypoint(geo.lat, geo.lon, item.z, item.u);
  });

  // Re-attach existing dive waypoints if any
  if (existingDiveWpts.length > 0) {
    const lastCalibE = calibNeds[calibNeds.length - 1].e;
    const firstDiveE = existingDiveWpts[0].east;
    const shiftE = (lastCalibE + 200) - firstDiveE;

    existingDiveWpts.forEach(w => {
      const shiftedE = w.east + shiftE;
      const geo = nedToLatLon(w.north, shiftedE, originLat, originLon);
      addWaypoint(geo.lat, geo.lon, w.depth, w.speed);
    });
  } else {
    // Add standard dive mission points following calibration
    const lastCalibN = calibNeds[calibNeds.length - 1].n;
    const lastCalibE = calibNeds[calibNeds.length - 1].e;
    const defaultDive = [
      { n: lastCalibN - 20,  e: lastCalibE + 200,  z: 10,  u: 1.5 },
      { n: lastCalibN - 100, e: lastCalibE + 600,  z: 100, u: 1.5 },
      { n: lastCalibN,       e: lastCalibE + 950,  z: 100, u: 1.5 },
      { n: lastCalibN + 200, e: lastCalibE + 1300, z: 50,  u: 1.5 },
      { n: lastCalibN + 200, e: lastCalibE + 1800, z: 50,  u: 1.5 },
      { n: lastCalibN + 400, e: lastCalibE + 2200, z: 50,  u: 1.5 }
    ];
    defaultDive.forEach(item => {
      const geo = nedToLatLon(item.n, item.e, originLat, originLon);
      addWaypoint(geo.lat, geo.lon, item.z, item.u);
    });
  }

  closeCalibrationModal();
  zoomToCalibrationZone();
}

/**
 * Load Default 12-Waypoint Remus 100 Scenario placed 100% on Lan Ha Bay water
 */
function loadDefaultScenario() {
  waypoints.forEach(w => map.removeLayer(w.marker));
  waypoints = [];

  const preset = REGION_PRESETS.lanha;
  originLat = preset.lat;
  originLon = preset.lon;

  const defaultNeds = [
    { n: 0,    e: 0,    z: 0,   u: 1.5 },
    { n: 0,    e: 120,  z: 0,   u: 1.5 },
    { n: -30,  e: 270,  z: 0,   u: 1.5 },
    { n: 30,   e: 430,  z: 0,   u: 1.5 },
    { n: -20,  e: 580,  z: 0,   u: 1.5 },
    { n: 0,    e: 720,  z: 0,   u: 1.5 },
    { n: -20,  e: 920,  z: 10,  u: 1.5 },
    { n: -100, e: 1320, z: 100, u: 1.5 },
    { n: 0,    e: 1670, z: 100, u: 1.5 },
    { n: 200,  e: 2020, z: 50,  u: 1.5 },
    { n: 200,  e: 2520, z: 50,  u: 1.5 },
    { n: 400,  e: 2920, z: 50,  u: 1.5 }
  ];

  defaultNeds.forEach(item => {
    const geo = nedToLatLon(item.n, item.e, originLat, originLon);
    addWaypoint(geo.lat, geo.lon, item.z, item.u);
  });

  currentHeadingDeg = 90;
  if (document.getElementById('headingSlider')) {
    document.getElementById('headingSlider').value = 90;
    document.getElementById('headingDisplay').innerText = '090° (Đông)';
  }

  if (waypoints.length > 0) {
    const bounds = L.latLngBounds(waypoints.map(w => [w.lat, w.lon]));
    map.fitBounds(bounds, { padding: [50, 50] });
  }
}

/**
 * Update Full UI State
 */
function updateUI() {
  renderPolylines();
  renderDepthProfile();
  updateTable();
  updateMetrics();
}

/**
 * Generate Clean MATLAB Script Code
 */
function generateMatlabScript() {
  if (waypoints.length === 0) {
    return '% Chưa có Waypoint nào được định nghĩa.';
  }

  const xList = waypoints.map(w => `${w.north.toFixed(1).padStart(7)}`).join(',');
  const yList = waypoints.map(w => `${w.east.toFixed(1).padStart(7)}`).join(',');
  const zList = waypoints.map(w => `${w.depth.toFixed(1).padStart(7)}`).join(',');
  const uList = waypoints.map(w => `${w.speed.toFixed(2).padStart(7)}`).join(',');

  let totalDist = 0;
  let totalTime = 0;
  let cutTime = 0;
  let cutFound = false;

  for (let i = 1; i < waypoints.length; i++) {
    const pPrev = waypoints[i - 1];
    const p = waypoints[i];
    const d = Math.sqrt(Math.pow(p.north - pPrev.north, 2) + Math.pow(p.east - pPrev.east, 2) + Math.pow(p.depth - pPrev.depth, 2));
    const avgSpeed = (p.speed + pPrev.speed) / 2 || 1.5;
    const dt = d / avgSpeed;
    totalDist += d;
    totalTime += dt;

    if (!cutFound && p.depth > 0.5) {
      const frac = (0.5 - pPrev.depth) / (p.depth - pPrev.depth);
      cutTime = (totalTime - dt) + Math.max(0, frac * dt);
      cutFound = true;
    }
  }

  if (!cutFound) cutTime = totalTime;
  const finalTime = Math.ceil(totalTime * 1.35 + 300); // Dự phòng thêm 35% và 300s quay đầu/dòng ngược (mô phỏng sẽ tự ngắt sớm ngay khi chạm đích cuối)

  const code = `% =========================================================================
% KỊCH BẢN QUỸ ĐẠO NHIỆM VỤ REMUS 100 (TỰ ĐỘNG SINH TỪ MISSION PLANNER)
% Ngày tạo: ${new Date().toLocaleString('vi-VN')}
% Tọa độ gốc WGS-84 (WP1):
%   Vĩ độ (Lat0) = ${originLat.toFixed(6)} deg
%   Kinh độ (Lon0) = ${originLon.toFixed(6)} deg
% =========================================================================

% 1. Định nghĩa ma trận Waypoints chuẩn 3D cho bộ dẫn đường ALOS3D
wpt.pos.x = [${xList} ]; % North (m)
wpt.pos.y = [${yList} ]; % East (m)
wpt.pos.z = [${zList} ]; % Down (m)
wpt.speed = [${uList} ]; % Tốc độ hành trình đặt (m/s)

% 2. Cấu hình thời gian chạy mô phỏng
T_final    = ${finalTime};       % Tổng thời gian mô phỏng (giây) [bao gồm dự phòng quay đầu]
T_gnss_cut = ${cutTime.toFixed(1)};      % Thời điểm cắt GNSS (giây) [khi tàu lặn z > 0.5m]

% 3. Tọa độ địa lý gốc WGS-84
lat_origin_deg = ${originLat.toFixed(6)};
lon_origin_deg = ${originLon.toFixed(6)};
lat_rad        = deg2rad(lat_origin_deg);

fprintf('[KỊCH BẢN] Đã nạp thành công %d Waypoints.\\n', length(wpt.pos.x));
fprintf('          - Tổng cự ly hành trình: %.2f km\\n', ${ (totalDist / 1000).toFixed(2) });
fprintf('          - Thời gian ước tính    : %.1f s (~%.1f phút)\\n', ${totalTime.toFixed(1)}, ${ (totalTime / 60).toFixed(1) });
fprintf('          - Thời điểm cắt GNSS    : %.1f s\\n', T_gnss_cut);
`;
  return code;
}

function openMatlabModal() {
  const code = generateMatlabScript();
  document.getElementById('matlabCodeBox').innerText = code;
  document.getElementById('codeModal').classList.add('open');
}

function closeMatlabModal() {
  document.getElementById('codeModal').classList.remove('open');
}

async function downloadMatlabFile() {
  const code = generateMatlabScript();

  // 1. Uu tien dung File System Access API cua trinh duyet (hien hop thoai Save As... cua Windows)
  if ('showSaveFilePicker' in window) {
    try {
      const handle = await window.showSaveFilePicker({
        suggestedName: 'kich_ban_mission.m',
        types: [{
          description: 'MATLAB Script (*.m)',
          accept: { 'text/plain': ['.m'] }
        }]
      });
      const writable = await handle.createWritable();
      await writable.write(code);
      await writable.close();
      alert('Đã lưu thành công file: kich_ban_mission.m\nBạn có thể mở MATLAB chạy file run_fossen_esekf_integrated.m ngay!');
      return;
    } catch (err) {
      if (err.name === 'AbortError') return; // Nguoi dung bam Huy
    }
  }

  // 2. Fallback: Dung Data URI de Chrome khong tu dong tao ma UUID dai loang ngoang
  const encodedUri = 'data:text/plain;charset=utf-8,' + encodeURIComponent(code);
  const a = document.createElement('a');
  a.href = encodedUri;
  a.download = 'kich_ban_mission.m';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);

  alert('Đã tải thành công file kịch bản: kich_ban_mission.m\nFile nằm trong thư mục Downloads của bạn.\nMATLAB sẽ tự động nạp file này khi chạy!');
}

function copyMatlabCode() {
  const code = generateMatlabScript();
  navigator.clipboard.writeText(code).then(() => {
    alert('Đã sao chép mã MATLAB vào Clipboard thành công!');
  }).catch(err => {
    alert('Không thể sao chép tự động: ' + err);
  });
}

function saveProjectJson() {
  const data = {
    originLat: originLat,
    originLon: originLon,
    currentHeadingDeg: currentHeadingDeg,
    waypoints: waypoints.map(w => ({
      id: w.id,
      lat: w.lat,
      lon: w.lon,
      north: w.north,
      east: w.east,
      depth: w.depth,
      speed: w.speed
    }))
  };

  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'remus100_mission.json';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

function loadProjectJson(event) {
  const file = event.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = function(e) {
    try {
      const data = JSON.parse(e.target.result);
      if (data.originLat && data.originLon && Array.isArray(data.waypoints)) {
        originLat = data.originLat;
        originLon = data.originLon;
        if (data.currentHeadingDeg !== undefined) {
          currentHeadingDeg = data.currentHeadingDeg;
          document.getElementById('headingSlider').value = currentHeadingDeg;
          document.getElementById('headingDisplay').innerText = `${Math.round(currentHeadingDeg).toString().padStart(3, '0')}° (${getCompassDirection(currentHeadingDeg)})`;
        }
        waypoints.forEach(w => map.removeLayer(w.marker));
        waypoints = [];

        data.waypoints.forEach(w => {
          addWaypoint(w.lat, w.lon, w.depth, w.speed);
        });

        if (waypoints.length > 0) {
          map.fitBounds(L.latLngBounds(waypoints.map(w => [w.lat, w.lon])));
        }
        alert(`Đã nạp thành công kịch bản gồm ${data.waypoints.length} Waypoints!`);
      } else {
        alert('Cấu trúc file JSON không hợp lệ.');
      }
    } catch (err) {
      alert('Lỗi đọc file JSON: ' + err);
    }
  };
  reader.readAsText(file);
}

window.addEventListener('resize', () => {
  renderDepthProfile();
});

document.addEventListener('DOMContentLoaded', () => {
  initMap();
});
