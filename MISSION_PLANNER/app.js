/**
 * ==========================================================================
 * AUV REMUS 100 MISSION PLANNER - APPLICATION ENGINE
 * High-Precision WGS-84 Geodesy, Interactive Leaflet Map & MATLAB Generator
 * ==========================================================================
 */

// WGS-84 Ellipsoid Constants
const WGS84_A = 6378137.0;            // Semi-major axis (meters)
const WGS84_F = 1 / 298.257223563;    // Flattening
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared (~0.00669438)

// Preset Regions
const REGION_PRESETS = {
  catba: {
    name: 'Vịnh Hạ Long / Cát Bà (Hải Phòng)',
    lat: 20.800000,
    lon: 107.050000,
    zoom: 14
  },
  hoabinh: {
    name: 'Hồ Hòa Bình (Vùng đo sâu thực tế)',
    lat: 20.758000,
    lon: 105.062000,
    zoom: 14
  },
  camranh: {
    name: 'Vịnh Cam Ranh (Khánh Hòa)',
    lat: 11.950000,
    lon: 109.180000,
    zoom: 13
  },
  vungtau: {
    name: 'Vùng biển Vũng Tàu',
    lat: 10.350000,
    lon: 107.080000,
    zoom: 13
  }
};

// Global State
let map = null;
let currentBasemap = null;
let originLat = REGION_PRESETS.catba.lat;
let originLon = REGION_PRESETS.catba.lon;
let waypoints = []; // Array of Waypoint objects
let pathPolylines = [];
let segmentLabels = [];
let selectedWptId = null;
let addModeActive = true;

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
  const preset = REGION_PRESETS.catba;
  map = L.map('map', {
    center: [preset.lat, preset.lon],
    zoom: preset.zoom,
    zoomControl: false
  });

  // Zoom control in bottom right
  L.control.zoom({ position: 'bottomright' }).addTo(map);

  // Basemap definitions
  const basemaps = {
    'googleHybrid': L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
      maxZoom: 20,
      attribution: '© Google Maps Satellite'
    }),
    'googleSat': L.tileLayer('https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
      maxZoom: 20,
      attribution: '© Google Satellite'
    }),
    'esriSat': L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 19,
      attribution: '© Esri World Imagery'
    }),
    'cartoDark': L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      maxZoom: 19,
      attribution: '© CartoDB Dark'
    })
  };

  // Add default basemap
  currentBasemap = basemaps.googleHybrid;
  currentBasemap.addTo(map);

  // Map Basemap Selector Event
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
        recalculateAllNed();
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

  // Map Click to Add Waypoint
  map.on('click', (e) => {
    if (!addModeActive) return;
    addWaypoint(e.latlng.lat, e.latlng.lng);
  });

  // Auto Load Default 12-Waypoint Remus 100 Scenario on Start
  loadDefaultScenario();
}

/**
 * Add a New Waypoint
 */
function addWaypoint(lat, lon, depth = null, speed = 1.5) {
  const id = waypoints.length + 1;
  
  // Set default origin to WP1 if it's the first waypoint
  if (waypoints.length === 0) {
    originLat = lat;
    originLon = lon;
  }

  const ned = latLonToNed(lat, lon, originLat, originLon);

  // Determine default depth: first 6 points surface (0m), subsequent points dive
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
}

/**
 * Create or Recreate Leaflet Marker for a Waypoint
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
    iconSize: [32, 32],
    iconAnchor: [16, 16]
  });

  const marker = L.marker([wpt.lat, wpt.lon], {
    icon: icon,
    draggable: true
  }).addTo(map);

  marker.on('drag', (e) => {
    const pos = marker.getLatLng();
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

  marker.on('click', () => {
    selectWaypoint(wpt.id);
  });

  wpt.marker = marker;
}

/**
 * Re-render Map Polylines with GNSS / Underwater Color-Coding
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
    const dashArray = isSurfaceSegment ? null : '8, 8';

    const polyline = L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], {
      color: color,
      weight: 3.5,
      opacity: 0.9,
      dashArray: dashArray
    }).addTo(map);
    pathPolylines.push(polyline);

    // Midpoint distance label
    const midLat = (p1.lat + p2.lat) / 2;
    const midLon = (p1.lon + p2.lon) / 2;
    const dx = p2.north - p1.north;
    const dy = p2.east - p1.east;
    const dz = p2.depth - p1.depth;
    const dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const headingDeg = (Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360;

    const labelIcon = L.divIcon({
      className: '',
      html: `<div style="background: rgba(10,17,32,0.85); color: #94a3b8; border: 1px solid rgba(56,189,248,0.25); padding: 2px 6px; border-radius: 4px; font-size: 9px; font-family: monospace; white-space: nowrap; transform: translate(-50%, -50%);">
              <span style="color:#00f0ff;">${dist3D.toFixed(0)}m</span> | ${headingDeg.toFixed(0)}°
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
  
  // High DPI scaling
  const width = canvas.parentElement.clientWidth;
  const height = canvas.parentElement.clientHeight;
  canvas.width = width * window.devicePixelRatio;
  canvas.height = height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

  ctx.clearRect(0, 0, width, height);

  if (waypoints.length === 0) {
    ctx.fillStyle = '#64748b';
    ctx.font = '12px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('Chưa có Waypoint nào. Click trên bản đồ để thêm mốc hành trình.', width / 2, height / 2);
    return;
  }

  // Calculate cumulative distances
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
  maxDepth = Math.max(maxDepth, 30); // minimum scale 30m

  const totalDist = dists[dists.length - 1];
  const padding = { left: 50, right: 30, top: 20, bottom: 25 };
  const graphWidth = width - padding.left - padding.right;
  const graphHeight = height - padding.top - padding.bottom;

  // Surface water line
  const surfaceY = padding.top;

  // Draw grid & depth labels
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#64748b';
  ctx.font = '10px JetBrains Mono';
  ctx.textAlign = 'right';

  const depthSteps = [0, 20, 50, 100, 150];
  depthSteps.forEach(d => {
    if (d <= maxDepth * 1.15) {
      const y = padding.top + (d / (maxDepth * 1.15)) * graphHeight;
      ctx.beginPath();
      ctx.moveTo(padding.left, y);
      ctx.lineTo(width - padding.right, y);
      ctx.stroke();
      ctx.fillText(`${d}m`, padding.left - 8, y + 3);
    }
  });

  // Sea surface label
  ctx.fillStyle = '#10b981';
  ctx.fillText('Mặt nước (z=0)', padding.left + 90, surfaceY - 5);

  if (totalDist <= 0) return;

  // Compute screen coordinates for each waypoint
  const coords = waypoints.map((wpt, idx) => {
    const x = padding.left + (dists[idx] / totalDist) * graphWidth;
    const y = padding.top + (wpt.depth / (maxDepth * 1.15)) * graphHeight;
    return { x, y, wpt };
  });

  // Ocean fill gradient under trajectory
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

  // Draw Trajectory Line
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

  // Draw Waypoint Points & Labels
  coords.forEach((c, idx) => {
    const isSurface = c.wpt.depth <= 0.5;
    ctx.beginPath();
    ctx.arc(c.x, c.y, 5, 0, Math.PI * 2);
    ctx.fillStyle = isSurface ? '#10b981' : '#00f0ff';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Text callout
    ctx.fillStyle = '#f8fafc';
    ctx.font = '10px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(`WP${idx + 1} (${c.wpt.depth}m)`, c.x, c.y - 10);
  });
}

/**
 * Update Sidebar Waypoint Table
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
      ? '<span class="wpt-tag wpt-tag-surface">Nổi (GNSS)</span>'
      : '<span class="wpt-tag wpt-tag-dive">Lặn (DVL)</span>';

    tr.innerHTML = `
      <td style="font-weight:700; color:var(--accent-cyan);">WP${wpt.id}</td>
      <td>${wpt.north >= 0 ? '+' : ''}${wpt.north.toFixed(0)}m</td>
      <td>${wpt.east >= 0 ? '+' : ''}${wpt.east.toFixed(0)}m</td>
      <td>
        <input type="number" step="1" min="0" max="200" class="wpt-input" value="${wpt.depth}"
          onchange="updateWptDepth(${wpt.id}, this.value)">
      </td>
      <td>
        <input type="number" step="0.1" min="0.5" max="3.0" class="wpt-input" value="${wpt.speed}"
          onchange="updateWptSpeed(${wpt.id}, this.value)">
      </td>
      <td>${tagHtml}</td>
      <td style="text-align:center;">
        <button class="btn btn-danger btn-sm" onclick="deleteWaypoint(${wpt.id}, event)" title="Xóa waypoint">×</button>
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

/**
 * Fast table update without losing focus when dragging
 */
function updateTableRowsOnly() {
  waypoints.forEach(wpt => {
    const row = document.getElementById(`wpt-row-${wpt.id}`);
    if (row) {
      const cells = row.querySelectorAll('td');
      if (cells.length >= 3) {
        cells[1].innerText = `${wpt.north >= 0 ? '+' : ''}${wpt.north.toFixed(0)}m`;
        cells[2].innerText = `${wpt.east >= 0 ? '+' : ''}${wpt.east.toFixed(0)}m`;
      }
    }
  });
}

/**
 * Update Mission Telemetry Metrics
 */
function updateMetrics() {
  let totalDist = 0;
  let totalTime = 0;
  let maxDepth = 0;
  let cutTime = 0;
  let cutFound = false;

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

      // Check GNSS cut instant
      if (!cutFound && p.depth > 0.5) {
        // interpolate cut instant along this segment
        const frac = (0.5 - pPrev.depth) / (p.depth - pPrev.depth);
        cutTime = (totalTime - segTime) + Math.max(0, frac * segTime);
        cutFound = true;
      }
    }
  }

  if (!cutFound && waypoints.length > 0 && waypoints[0].depth <= 0.5) {
    cutTime = totalTime; // entire mission on surface
  }

  document.getElementById('metricWptCount').innerText = waypoints.length;
  document.getElementById('metricDistance').innerHTML = `${(totalDist / 1000).toFixed(2)} <span class="metric-unit">km</span>`;
  
  const minutes = Math.floor(totalTime / 60);
  const seconds = Math.floor(totalTime % 60);
  document.getElementById('metricTime').innerHTML = `${minutes}m ${seconds}s <span class="metric-unit">(${totalTime.toFixed(0)}s)</span>`;
  
  document.getElementById('metricMaxDepth').innerHTML = `${maxDepth.toFixed(0)} <span class="metric-unit">m</span>`;
  document.getElementById('metricGnssCut').innerHTML = `${cutTime.toFixed(1)} <span class="metric-unit">giây</span>`;
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
  if (row) row.classList.add('active');
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
    
    // Re-index remaining waypoints
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

function recalculateAllNed() {
  if (waypoints.length === 0) return;
  originLat = waypoints[0].lat;
  originLon = waypoints[0].lon;

  waypoints.forEach(w => {
    const ned = latLonToNed(w.lat, w.lon, originLat, originLon);
    w.north = ned.north;
    w.east = ned.east;
  });
  updateUI();
}

/**
 * Load Default 12-Waypoint Remus 100 Scenario
 */
function loadDefaultScenario() {
  waypoints.forEach(w => map.removeLayer(w.marker));
  waypoints = [];

  const baseLat = originLat;
  const baseLon = originLon;

  // 12 standard NED waypoints from run_fossen_esekf_integrated.m
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
    const geo = nedToLatLon(item.n, item.e, baseLat, baseLon);
    addWaypoint(geo.lat, geo.lon, item.z, item.u);
  });

  // Fit bounds to show entire trajectory
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

  // Calculate mission duration and GNSS cut time
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

  // Add 10% time buffer for turning maneuvers
  const finalTime = Math.ceil(totalTime * 1.15);

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

/**
 * Open MATLAB Script Preview Modal
 */
function openMatlabModal() {
  const code = generateMatlabScript();
  document.getElementById('matlabCodeBox').innerText = code;
  document.getElementById('codeModal').classList.add('open');
}

function closeMatlabModal() {
  document.getElementById('codeModal').classList.remove('open');
}

/**
 * Download .m script file
 */
function downloadMatlabFile() {
  const code = generateMatlabScript();
  const blob = new Blob([code], { type: 'text/plain;charset=utf-8' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'kich_ban_mission.m';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

/**
 * Copy MATLAB script to Clipboard
 */
function copyMatlabCode() {
  const code = generateMatlabScript();
  navigator.clipboard.writeText(code).then(() => {
    alert('Đã sao chép mã MATLAB vào Clipboard thành công!');
  }).catch(err => {
    alert('Không thể sao chép tự động: ' + err);
  });
}

/**
 * Save Project to JSON
 */
function saveProjectJson() {
  const data = {
    originLat: originLat,
    originLon: originLon,
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

/**
 * Load Project from JSON file input
 */
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

// Window Resize Hook
window.addEventListener('resize', () => {
  renderDepthProfile();
});

// Initialize on DOM Ready
document.addEventListener('DOMContentLoaded', () => {
  initMap();
});
