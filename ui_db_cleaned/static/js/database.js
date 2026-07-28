function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>'"]/g, character => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    "'": '&#39;',
    '"': '&quot;',
  }[character]));
}

function prettyJson(value) {
  return JSON.stringify(value, null, 2);
}

let objectCache = new Map();

async function fetchJson(url, options = {}) {
  const response = await fetch(url, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.message || `HTTP ${response.status}`);
  }
  return data;
}

async function loadHealth() {
  const badge = document.getElementById('redisStatus');
  try {
    await fetchJson('/api/health');
    badge.textContent = 'Redis 연결됨';
    badge.className = 'connection-badge connected';
  } catch (error) {
    badge.textContent = 'Redis 연결 실패';
    badge.className = 'connection-badge error';
  }
}

function renderFixedPoint(point) {
  return `
    <article class="pose-card">
      <div><strong>${escapeHtml(point.name)}</strong><span>고정</span></div>
      <dl>
        <dt>X</dt><dd>${Number(point.x).toFixed(2)} mm</dd>
        <dt>Y</dt><dd>${Number(point.y).toFixed(2)} mm</dd>
        <dt>Z</dt><dd>${Number(point.z).toFixed(2)} mm</dd>
        <dt>RX</dt><dd>${Number(point.rx).toFixed(2)}°</dd>
        <dt>RY</dt><dd>${Number(point.ry).toFixed(2)}°</dd>
        <dt>RZ</dt><dd>${Number(point.rz).toFixed(2)}°</dd>
      </dl>
      <code>assistive_robot:fixed_point:${escapeHtml(point.name)}</code>
    </article>`;
}

function renderScanCase(scanCase) {
  const sequence = (scanCase.waypoint_names || [])
    .map((name, index) => `<span><b>${index + 1}</b>${escapeHtml(name)}</span>`)
    .join('<i>→</i>');

  const resolved = (scanCase.waypoints || []).map(waypoint => `
    <li>
      <strong>${waypoint.order}. ${escapeHtml(waypoint.name)}</strong>
      <code>[${waypoint.pose.map(value => Number(value).toFixed(2)).join(', ')}]</code>
    </li>`).join('');

  return `
    <article class="case-card">
      <div class="case-title"><strong>${escapeHtml(scanCase.name)}</strong><span>Redis List</span></div>
      <div class="case-sequence">${sequence}</div>
      <ol>${resolved}</ol>
      <code>${escapeHtml(scanCase.redis_key)}</code>
    </article>`;
}

async function loadFixedConfig() {
  const data = await fetchJson('/api/admin/fixed-config');
  document.getElementById('fixedConfigVersion').textContent = `설정 버전: ${data.version || '-'}`;

  const pointGrid = document.getElementById('fixedPointGrid');
  pointGrid.innerHTML = (data.fixed_points || []).map(renderFixedPoint).join('')
    || '<p class="empty">고정 웨이포인트가 없습니다.</p>';

  const caseGrid = document.getElementById('scanCaseGrid');
  caseGrid.innerHTML = (data.scan_cases || []).map(renderScanCase).join('')
    || '<p class="empty">이동 케이스가 없습니다.</p>';
}

function renderObjectCard(object) {
  const encodedName = encodeURIComponent(object.record_name);
  return `
    <article class="object-card">
      <div class="object-card-head">
        <div>
          <strong>${escapeHtml(object.record_name)}</strong>
          <code>${escapeHtml(object.redis_key)}</code>
        </div>
        <div class="object-actions">
          <button class="secondary" data-action="edit" data-name="${escapeHtml(encodedName)}">편집</button>
          <button class="danger" data-action="delete" data-name="${escapeHtml(encodedName)}">삭제</button>
        </div>
      </div>
      <pre class="json-view">${escapeHtml(prettyJson(object.data || {}))}</pre>
    </article>`;
}

async function loadObjects() {
  const data = await fetchJson('/api/admin/objects');
  const objects = data.objects || [];
  objectCache = new Map(objects.map(object => [object.record_name, object]));

  const grid = document.getElementById('objectGrid');
  grid.innerHTML = objects.map(renderObjectCard).join('')
    || '<p class="empty">저장된 인식 객체가 없습니다.</p>';
}

function clearObjectForm() {
  const form = document.getElementById('objectForm');
  form.reset();
  form.elements.record_name.value = '';
  form.elements.data.value = '{}';
  form.elements.replace.checked = true;
  const message = document.getElementById('formMessage');
  message.textContent = '';
  message.className = 'wide';
}

function editObject(recordName) {
  const object = objectCache.get(recordName);
  if (!object) return;
  const form = document.getElementById('objectForm');
  form.elements.record_name.value = object.record_name;
  form.elements.data.value = prettyJson(object.data || {});
  form.elements.replace.checked = true;
  form.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

async function deleteObject(recordName) {
  if (!confirm(`'${recordName}' 객체 레코드를 삭제할까요?`)) return;
  await fetchJson(`/api/admin/objects/${encodeURIComponent(recordName)}`, {
    method: 'DELETE',
  });
  await loadObjects();
}

document.getElementById('objectGrid').addEventListener('click', event => {
  const button = event.target.closest('button[data-action]');
  if (!button) return;
  const recordName = decodeURIComponent(button.dataset.name);
  if (button.dataset.action === 'edit') editObject(recordName);
  if (button.dataset.action === 'delete') {
    deleteObject(recordName).catch(error => alert(error.message));
  }
});

document.getElementById('refreshButton').addEventListener('click', async () => {
  await Promise.all([loadHealth(), loadFixedConfig(), loadObjects()]);
});

document.getElementById('clearFormButton').addEventListener('click', clearObjectForm);

document.getElementById('objectForm').addEventListener('submit', async event => {
  event.preventDefault();
  const form = event.currentTarget;
  const formData = new FormData(form);
  const message = document.getElementById('formMessage');

  let objectData;
  try {
    objectData = JSON.parse(formData.get('data') || '{}');
    if (!objectData || Array.isArray(objectData) || typeof objectData !== 'object') {
      throw new Error('JSON 최상위 값은 객체여야 합니다.');
    }
    if (Object.keys(objectData).length === 0) {
      throw new Error('최소 한 개 이상의 필드를 입력하세요.');
    }
  } catch (error) {
    message.textContent = `JSON 형식을 확인하세요: ${error.message}`;
    message.className = 'error wide';
    return;
  }

  const payload = {
    record_name: formData.get('record_name'),
    data: objectData,
    replace: form.elements.replace.checked,
  };

  try {
    await fetchJson('/api/admin/objects', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    message.textContent = payload.replace
      ? '객체 레코드를 전체 교체해 저장했습니다.'
      : '기존 객체 레코드에 필드를 병합했습니다.';
    message.className = 'success wide';
    await loadObjects();
  } catch (error) {
    message.textContent = error.message || '저장에 실패했습니다.';
    message.className = 'error wide';
  }
});

Promise.all([loadHealth(), loadFixedConfig(), loadObjects()]).catch(error => {
  console.error(error);
});
setInterval(loadHealth, 5000);
setInterval(() => loadObjects().catch(console.error), 2000);

function renderConversationItem(item) {
  const role = String(item.role || 'unknown').toUpperCase();
  return `
    <article class="log-item conversation-item">
      <div class="log-meta">
        <strong>${escapeHtml(role)}</strong>
        <span>${escapeHtml(item.timestamp || '')}</span>
        <code>${escapeHtml(item.source || '')}</code>
      </div>
      <p>${escapeHtml(item.text || '')}</p>
    </article>`;
}

async function loadConversations() {
  const data = await fetchJson('/api/admin/conversations?limit=500');
  const container = document.getElementById('conversationLog');
  const items = data.conversations || [];
  container.innerHTML = items.map(renderConversationItem).join('')
    || '<p class="empty">저장된 VLA 대화가 없습니다.</p>';
  container.scrollTop = container.scrollHeight;
}

function renderRuntimeLogItem(item) {
  const level = String(item.level || 'INFO').toUpperCase();
  const details = item.details
    ? `<pre>${escapeHtml(prettyJson(item.details))}</pre>`
    : '';
  return `
    <article class="log-item runtime-item" data-level="${escapeHtml(level)}">
      <div class="log-meta">
        <strong>${escapeHtml(level)}</strong>
        <span>${escapeHtml(item.timestamp || '')}</span>
        <code>${escapeHtml(item.source || '')}</code>
        <em>${escapeHtml(item.category || '')}</em>
      </div>
      <p>${escapeHtml(item.message || '')}</p>
      ${details}
    </article>`;
}

async function loadRuntimeLogs() {
  const data = await fetchJson('/api/admin/runtime-logs?limit=500');
  const container = document.getElementById('runtimeLog');
  const items = data.logs || [];
  container.innerHTML = items.map(renderRuntimeLogItem).join('')
    || '<p class="empty">기록된 런타임 로그가 없습니다.</p>';
  container.scrollTop = container.scrollHeight;
}

async function clearConversations() {
  if (!confirm('저장된 VLA 대화 기록을 모두 삭제할까요?')) return;
  await fetchJson('/api/admin/conversations', { method: 'DELETE' });
  await loadConversations();
}

async function clearRuntimeLogs() {
  if (!confirm('로컬 런타임 로그를 모두 삭제할까요?')) return;
  await fetchJson('/api/admin/runtime-logs', { method: 'DELETE' });
  await loadRuntimeLogs();
}

document.getElementById('refreshConversationButton').addEventListener('click', () => {
  loadConversations().catch(error => alert(error.message));
});
document.getElementById('clearConversationButton').addEventListener('click', () => {
  clearConversations().catch(error => alert(error.message));
});
document.getElementById('refreshRuntimeButton').addEventListener('click', () => {
  loadRuntimeLogs().catch(error => alert(error.message));
});
document.getElementById('clearRuntimeButton').addEventListener('click', () => {
  clearRuntimeLogs().catch(error => alert(error.message));
});

loadConversations().catch(console.error);
loadRuntimeLogs().catch(console.error);
setInterval(() => loadConversations().catch(console.error), 2000);
setInterval(() => loadRuntimeLogs().catch(console.error), 2000);
