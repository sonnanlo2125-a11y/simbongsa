const STATE_VIEW = {
  idle: {
    title: '말씀해 주세요',
    hint: '두팔이가 기다리고 있어요',
    aria: '대기 중',
  },
  listening: {
    title: '듣고 있어요',
    hint: '편하게 말씀해 주세요',
    aria: '사용자의 말을 듣는 중',
  },
  thinking: {
    title: '생각하고 있어요',
    hint: '요청을 이해하는 중이에요',
    aria: '요청을 처리하는 중',
  },
  speaking: {
    title: '두팔이가 말하고 있어요',
    hint: '잠시만 들어 주세요',
    aria: '로봇이 대답하는 중',
  },
  error: {
    title: '연결을 확인해 주세요',
    hint: '잠시 후 다시 시도할게요',
    aria: '서버 연결 오류',
  },
};

const body = document.body;
const waveBars = document.getElementById('waveBars');
const waveShell = document.getElementById('waveShell');
const voiceState = document.getElementById('voiceState');
const voiceHint = document.getElementById('voiceHint');
const connectionIndicator = document.getElementById('connectionIndicator');
const connectionText = document.getElementById('connectionText');

let lastState = '';
let lastUserText = '';
let lastAssistantText = '';

function buildWaveBars() {
  const barCount = 72;
  const fragment = document.createDocumentFragment();

  for (let index = 0; index < barCount; index += 1) {
    const bar = document.createElement('span');
    const centerDistance = Math.abs(index - (barCount - 1) / 2);
    const centerWeight = 1 - centerDistance / ((barCount - 1) / 2);
    const seed = 0.35 + Math.random() * 0.65;

    bar.className = 'wave-bar';
    bar.style.setProperty('--index', index);
    bar.style.setProperty('--center-weight', centerWeight.toFixed(3));
    bar.style.setProperty('--seed', seed.toFixed(3));
    bar.style.setProperty('--delay', `${(-index * 0.027).toFixed(3)}s`);
    fragment.appendChild(bar);
  }

  waveBars.replaceChildren(fragment);
}

function normalizeState(rawState) {
  const value = String(rawState || 'idle').trim().toLowerCase();

  if (['listening', 'listen', 'recording', 'hearing', 'user'].includes(value)) {
    return 'listening';
  }
  if (['thinking', 'processing', 'working', 'planning'].includes(value)) {
    return 'thinking';
  }
  if (['speaking', 'talking', 'responding', 'assistant', 'robot'].includes(value)) {
    return 'speaking';
  }
  if (['error', 'failed', 'disconnected', 'offline'].includes(value)) {
    return 'error';
  }
  return 'idle';
}

function triggerWaveBurst(type) {
  waveShell.classList.remove('wave-burst-user', 'wave-burst-robot');
  // Force the animation to restart when the same speaker sends consecutive text.
  void waveShell.offsetWidth;
  waveShell.classList.add(type === 'user' ? 'wave-burst-user' : 'wave-burst-robot');
}

function applyState(state, data = {}) {
  const view = STATE_VIEW[state] || STATE_VIEW.idle;
  body.dataset.state = state;
  voiceState.textContent = view.title;
  voiceHint.textContent = data.message || view.hint;
  waveShell.setAttribute('aria-label', view.aria);

  connectionIndicator.dataset.connected = state === 'error' ? 'false' : 'true';
  connectionText.textContent = state === 'error' ? '연결 끊김' : '연결됨';

  const nextUserText = String(data.user_text || '');
  const nextAssistantText = String(data.assistant_text || '');

  if (nextUserText && nextUserText !== lastUserText) {
    triggerWaveBurst('user');
  }
  if (nextAssistantText && nextAssistantText !== lastAssistantText) {
    triggerWaveBurst('robot');
  }

  lastUserText = nextUserText;
  lastAssistantText = nextAssistantText;
  lastState = state;
}

async function updateUserState() {
  try {
    const response = await fetch('/api/user/state', {
      cache: 'no-store',
      headers: { Accept: 'application/json' },
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const data = await response.json();
    applyState(normalizeState(data.state), data);
  } catch (error) {
    applyState('error', {});
  }
}

buildWaveBars();
updateUserState();
setInterval(updateUserState, 500);

window.addEventListener('focus', updateUserState);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) {
    updateUserState();
  }
});
