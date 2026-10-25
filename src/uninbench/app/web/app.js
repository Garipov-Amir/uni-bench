'use strict';
/* uninbench web UI — a dependency-free single-page app over the local JSON API. */

const TOKEN = document.querySelector('meta[name="uninbench-token"]').content;
const main = document.getElementById('main');
const SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948'];
const OUTCOME_COLORS = { success: '#2a78d6', incomplete: '#86b6ef', invalid: '#eb6834', timeout: '#eda100',
  crash: '#e34948', not_applicable: '#c3c2b7' };
let pollTimer = null;
let navTimer = null;

/* ---------------------------------------------------------------- helpers */
function h(tag, attrs, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === null || v === undefined || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else if (k === 'html') el.innerHTML = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const kid of kids.flat(Infinity)) {
    if (kid === null || kid === undefined || kid === false) continue;
    el.append(kid instanceof Node ? kid : document.createTextNode(String(kid)));
  }
  return el;
}
function put(el, ...kids) {
  el.replaceChildren(...kids.flat(Infinity).filter(k => k !== null && k !== undefined && k !== false)
    .map(k => (k instanceof Node ? k : document.createTextNode(String(k)))));
}
async function api(path, params) {
  const q = params ? '?' + new URLSearchParams(Object.entries(params).filter(([, v]) => v !== undefined && v !== null && v !== '')) : '';
  const r = await fetch(path + q);
  const data = await r.json().catch(() => ({ error: r.statusText }));
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
async function post(path, body) {
  const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Uninbench-Token': TOKEN },
    body: JSON.stringify(body || {}) });
  const data = await r.json().catch(() => ({ error: r.statusText }));
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
let toastTimer;
function toast(msg, bad) {
  const t = document.getElementById('toast');
  t.textContent = msg; t.classList.remove('hidden'); t.style.background = bad ? 'var(--bad)' : '';
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.add('hidden'), bad ? 7000 : 3500);
}
async function guard(fn) { try { return await fn(); } catch (e) { toast(e.message, true); } }
function fmt(v, nd = 3) {
  if (v === null || v === undefined || v === '') return '–';
  if (typeof v === 'number') {
    if (!isFinite(v)) return '∞';
    if (Number.isInteger(v)) return String(v);
    if (Math.abs(v) >= 1000) return v.toFixed(0);
    return v.toPrecision(nd + 1).replace(/\.?0+$/, '') ;
  }
  if (Array.isArray(v)) return v.map(x => fmt(x, nd)).join(', ');
  if (typeof v === 'object') return Object.entries(v).map(([k, x]) => `${k}=${fmt(x, nd)}`).join(' ');
  return String(v);
}
function ago(ts) {
  if (!ts) return '–';
  const s = Date.now() / 1000 - ts;
  if (s < 60) return `${Math.round(s)} s ago`;
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  if (s < 86400) return `${Math.round(s / 3600)} h ago`;
  return new Date(ts * 1000).toLocaleDateString();
}
function dur(s) {
  if (s === null || s === undefined) return '–';
  if (s < 60) return `${s.toFixed(0)} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`;
  return `${Math.floor(s / 3600)} h ${Math.round((s % 3600) / 60)} min`;
}
const STATE_BADGE = { done: 'b-ok', success: 'b-ok', running: 'b-info', queued: 'b-mute', failed: 'b-bad',
  cancelled: 'b-warn', lost: 'b-bad', incomplete: 'b-warn', invalid: 'b-bad', timeout: 'b-warn', crash: 'b-bad',
  not_applicable: 'b-mute' };
const badge = s => h('span', { class: `badge ${STATE_BADGE[s] || 'b-mute'}` }, String(s).replace('_', ' '));
const bar = f => h('div', { class: 'bar', title: `${Math.round((f || 0) * 100)} %` }, h('i', { style: `width:${Math.round((f || 0) * 100)}%` }));
function stack(outcomes) {
  const total = Object.values(outcomes || {}).reduce((a, b) => a + b, 0) || 1;
  return h('div', { class: 'stack', title: fmt(outcomes) }, Object.entries(outcomes || {}).map(([k, v]) =>
    h('i', { style: `width:${(100 * v / total).toFixed(2)}%;background:${OUTCOME_COLORS[k] || '#898781'}` })));
}
function table(cols, rows, opts = {}) {
  if (!rows.length) return h('div', { class: 'empty' }, opts.empty || 'Nothing here yet.');
  return h('div', { class: 'tw' }, h('table', {},
    h('thead', {}, h('tr', {}, cols.map(c => h('th', {}, c.label)))),
    h('tbody', {}, rows.map(r => h('tr', { class: [opts.onclick ? 'click' : '', opts.selected && opts.selected(r) ? 'sel' : ''].join(' '),
      onclick: opts.onclick ? () => opts.onclick(r) : null },
      cols.map(c => h('td', {}, c.render ? c.render(r) : fmt(r[c.key]))))))));
}
function field(label, input) { return h('label', { class: 'f' }, label, input); }
function input(name, value, attrs) { return h('input', { name, value: value ?? '', ...(attrs || {}) }); }
function select(name, options, value) {
  return h('select', { name }, options.map(o => { const [v, t] = Array.isArray(o) ? o : [o, o];
    return h('option', { value: v, selected: v === value }, t); }));
}
function formData(root) {
  const out = {};
  root.querySelectorAll('input,select,textarea').forEach(el => {
    if (!el.name) return;
    if (el.type === 'checkbox') {
      if (el.dataset.group) { (out[el.name] = out[el.name] || []); if (el.checked) out[el.name].push(el.value); }
      else out[el.name] = el.checked;
    } else out[el.name] = el.value;
  });
  return out;
}
function modal(title, body, actions) {
  const m = document.getElementById('modal'), card = document.getElementById('modal-card');
  put(card, h('h2', {}, title), body, h('div', { class: 'row', style: 'margin-top:16px;justify-content:flex-end' },
    h('button', { onclick: closeModal }, 'Cancel'), actions));
  m.classList.remove('hidden');
  const first = card.querySelector('input,select,textarea'); if (first) first.focus();
}
function closeModal() { document.getElementById('modal').classList.add('hidden'); }
document.getElementById('modal').addEventListener('click', e => { if (e.target.id === 'modal') closeModal(); });
document.addEventListener('keydown', e => { if (e.key === 'Escape') closeModal(); });
function go(hash) { if (location.hash === hash) render(); else location.hash = hash; }
function fileUrl(path) { return '/files/' + path.split('/').map(encodeURIComponent).join('/'); }
async function startJob(endpoint, body) {
  const job = await post(endpoint, body);
  closeModal(); toast(`Started: ${job.title}`); refreshNav();
  go(`#/jobs/${job.id}`);
}

/* ---------------------------------------------------------------- router */
const VIEWS = { dashboard, instances, planners, experiments, jobs, results };
async function render() {
  clearInterval(pollTimer);
  const [, view = 'dashboard', ...rest] = location.hash.replace(/^#/, '').split('/');
  const arg = rest.length ? decodeURIComponent(rest.join('/')) : null;
  document.querySelectorAll('#nav a').forEach(a => a.classList.toggle('active', a.dataset.view === view));
  try { await (VIEWS[view] || dashboard)(arg); }
  catch (e) { put(main, h('h1', {}, 'Something went wrong'), h('pre', { class: 'log' }, e.message)); }
}
window.addEventListener('hashchange', render);
async function refreshNav() {
  try {
    const s = await api('/api/summary');
    const pill = document.getElementById('nav-running');
    pill.textContent = s.running; pill.classList.toggle('hidden', !s.running);
    document.getElementById('ws-root').textContent = s.root;
  } catch (e) { /* server stopped */ }
}

/* ---------------------------------------------------------------- dashboard */
async function dashboard() {
  const s = await api('/api/summary');
  const kpi = (n, label, hash) => h('a', { class: 'kpi', href: hash, style: 'text-decoration:none;color:inherit' }, h('b', {}, n), h('span', {}, label));
  put(main, 
    h('h1', {}, 'Dashboard'), h('p', { class: 'sub' }, `Workspace ${s.root} · uninbench ${s.version}`),
    !s.initialized ? h('div', { class: 'banner' }, h('b', {}, 'This folder is not a uninbench workspace yet. '),
      'Initialize it to create data/, experiments/, results/ and an example experiment. ',
      h('button', { class: 'primary', onclick: () => guard(async () => { await post('/api/init', {}); toast('Workspace initialized'); render(); }) }, 'Initialize workspace')) : null,
    h('div', { class: 'kpis' }, kpi(s.instance_sets, 'instance sets', '#/instances'), kpi(s.instances, 'instances', '#/instances'),
      kpi(s.experiments, 'experiments', '#/experiments'), kpi(s.results, 'result sets', '#/results'), kpi(s.running, 'jobs running', '#/jobs')),
    h('h2', {}, 'Quick actions'),
    h('div', { class: 'row' },
      h('button', { class: 'primary', onclick: () => go('#/experiments/+new') }, 'New experiment'),
      h('button', { onclick: () => generateModal() }, 'Generate instances'),
      h('button', { onclick: () => importModal(s.datasets) }, 'Import dataset'),
      h('button', { onclick: () => guard(() => startJob('/api/validate', {})) }, 'Validate planners'),
      h('button', { onclick: () => guard(() => startJob('/api/demo', { out: 'demo' })) }, 'Run the demo')),
    h('h2', {}, 'Recent jobs'), jobsTable(s.jobs));
  pollTimer = setInterval(async () => { if (s.jobs.some(j => ['queued', 'running'].includes(j.state))) render(); }, 3000);
}
function jobsTable(list, selected) {
  return table([
    { label: 'Job', render: j => j.title }, { label: 'Kind', key: 'kind' }, { label: 'State', render: j => badge(j.state) },
    { label: 'Progress', render: j => j.progress ? h('div', { class: 'row' }, bar(j.progress.fraction), `${j.progress.done}/${j.progress.total}`) : (j.state === 'done' ? bar(1) : '–') },
    { label: 'Started', render: j => ago(j.started || j.created) }, { label: 'Elapsed', render: j => dur(j.elapsed) }],
  list, { onclick: j => go(`#/jobs/${j.id}`), selected: j => j.id === selected, empty: 'No jobs yet — start one from any page.' });
}

/* ---------------------------------------------------------------- instances */
async function instances(arg) {
  const [sets, s] = await Promise.all([api('/api/instance-sets'), api('/api/summary')]);
  let [setPath, file] = (arg || '').split('::');
  if (!setPath && sets.length) setPath = sets[0].path;
  const left = h('div', {},
    h('div', { class: 'row', style: 'margin-bottom:10px' }, h('button', { class: 'primary', onclick: () => generateModal() }, 'Generate…'),
      h('button', { onclick: () => importModal(s.datasets) }, 'Import…')),
    sets.length ? h('div', { class: 'list' }, sets.map(st => h('div', { class: `item ${st.path === setPath ? 'sel' : ''}`,
      onclick: () => go(`#/instances/${encodeURIComponent(st.path)}`) },
      h('div', { class: 't' }, st.name), h('div', { class: 'd' }, `${st.count} instances · ${fmt(st.families)} · ${fmt(st.splits)}`))))
      : h('div', { class: 'empty' }, 'No instance sets yet. Generate synthetic instances or import a dataset.'));
  const right = h('div', {});
  put(main, h('h1', {}, 'Instances'), h('p', { class: 'sub' }, 'Canonical instances in data/. Each set is a directory with an index of tuning/test splits.'),
    h('div', { class: 'grid2' }, left, right));
  if (!setPath) return;
  const rows = await api('/api/instances', { set: setPath });
  const filter = input('filter', '', { placeholder: 'Filter by id, family, split…', style: 'min-width:260px' });
  const tbl = h('div', {});
  const detail = h('div', {});
  const draw = () => {
    const q = filter.value.toLowerCase();
    const shown = rows.filter(r => !q || [r.id, r.family, r.split, fmt(r.factors)].join(' ').toLowerCase().includes(q));
    put(tbl, table([{ label: 'Instance', key: 'id' }, { label: 'Family', key: 'family' }, { label: 'Split', render: r => r.split || '–' },
      { label: 'Holes', key: 'holes' }, { label: 'Robots', key: 'robots' }, { label: 'Width [m]', key: 'width' }, { label: 'Factors', render: r => fmt(r.factors) }],
    shown, { onclick: r => go(`#/instances/${encodeURIComponent(setPath)}::${encodeURIComponent(r.file)}`), selected: r => r.file === file }));
  };
  filter.addEventListener('input', draw); draw();
  put(right, h('div', { class: 'row spread' }, h('h2', { style: 'margin:0' }, setPath),
    h('div', { class: 'row' }, filter, h('button', { onclick: () => splitModal(setPath) }, 'Re-split…'))), h('div', { style: 'height:10px' }), detail, tbl);
  if (file) {
    put(detail, h('div', { class: 'card', style: 'margin-bottom:14px' }, h('div', { class: 'muted' }, 'Loading instance…')));
    const d = await api('/api/instance', { file });
    put(detail, h('div', { class: 'card', style: 'margin-bottom:14px' },
      h('div', { class: 'row spread' }, h('h3', { style: 'margin:0' }, d.id), h('span', { class: 'muted' }, d.file)),
      h('div', { class: 'detail-grid' },
        preview(d),
        h('dl', { class: 'kv' }, [['family', d.family], ['hash', d.hash], ['split', d.split], ['robots', d.robots],
          ['working width', `${fmt(d.working_width)} m`], ['extent', `${fmt(d.extent[0])} × ${fmt(d.extent[1])} m`],
          ['free area', `${fmt(d.free_area)} m²`], ['coverable area F*', `${fmt(d.coverable_area)} m²`], ['holes', d.holes],
          ['T_LB', `${fmt(d.T_LB)} s`], ['δ_e', `${fmt(d.delta_e)} m`], ['factors', fmt(d.factors)], ['source', d.metadata.source]]
          .map(([k, v]) => [h('dt', {}, k), h('dd', {}, v ?? '–')])))));
  }
}
function preview(d) {
  const [x0, y0, x1, y1] = d.bounds, pad = Math.max(x1 - x0, y1 - y0) * 0.03;
  const W = x1 - x0 + 2 * pad, H = y1 - y0 + 2 * pad;
  const path = geom => geom.map(poly => poly.map(ring => 'M' + ring.map(([x, y]) => `${(x - x0 + pad).toFixed(3)} ${(y1 - y + pad).toFixed(3)}`).join('L') + 'Z').join('')).join('');
  const svg = `<svg class="preview" viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="instance preview">
    <rect x="0" y="0" width="${W}" height="${H}" fill="#c3c2b7"/>
    <path d="${path(d.free_geometry)}" fill="var(--surface)" fill-rule="evenodd"/>
    <path d="${path(d.coverable_geometry)}" fill="#2a78d6" fill-opacity="0.12" fill-rule="evenodd"/>
    ${d.team.map((r, k) => `<circle cx="${r.start[0] - x0 + pad}" cy="${y1 - r.start[1] + pad}" r="${Math.max(r.tool.width / 2, W / 150)}" fill="${SERIES[k % 8]}"/>`).join('')}
  </svg>`;
  return h('div', { html: svg + '<div class="muted" style="font-size:12px;margin-top:4px">grey: obstacles · white: free space · blue tint: coverable set F* · dots: robot starts</div>' });
}
function generateModal() {
  const body = h('div', {});
  const kind = select('kind', [['synthetic', 'Synthetic environments (IF5)'], ['safelog', 'SafeLog-like graphs (IF4)']], 'synthetic');
  const fields = h('div', { class: 'form' });
  const draw = () => {
    const syn = kind.value === 'synthetic';
    put(fields, 
      field('Output set (data/…)', input('out', syn ? 'data/IF5' : 'data/IF4')),
      field('Instances per combination', input('count', syn ? 5 : 1, { type: 'number', min: 1 })),
      field('Seed', input('seed', 0, { type: 'number' })),
      syn ? field('Sizes (width in m)', input('sizes', '20 40')) : field('Rows × cols', h('div', { class: 'row' }, input('rows', 20, { type: 'number', style: 'width:70px' }), '×', input('cols', 20, { type: 'number', style: 'width:70px' }))),
      field(syn ? 'Obstacle densities' : 'Edge densities (empty = 21-step sweep)', input('densities', syn ? '0.05 0.1' : '')),
      syn ? field('Obstacle shapes', input('shapes', 'mixed', { placeholder: 'rect polygon circle mixed' })) : null,
      syn ? field('Boundary notches', input('nonconvexity', '0')) : null,
      field('Tuning fraction', input('tuning_fraction', 0.2, { type: 'number', step: 0.05, min: 0, max: 1 })),
      field('Stratify split by factor', input('stratify', syn ? 'obstacle_density' : 'graph_density')));
  };
  kind.addEventListener('change', draw); draw();
  body.append(field('Generator', kind), h('div', { style: 'height:12px' }), fields);
  modal('Generate instances', body, h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/generate', formData(body))) }, 'Generate'));
}
function importModal(datasets) {
  const body = h('div', { class: 'form' },
    field('Dataset', select('dataset', Object.entries(datasets).map(([k, v]) => [k, `${k} — ${v}`]), 'areacoverage')),
    field('Source directory', input('src', 'datasets/AreaCoverage-dataset')),
    field('Output set', input('out', 'data/IF1')),
    field('Importer parameters (k=v …)', input('params', '', { placeholder: 'width=3.0 limit=20' })),
    field('Tuning fraction', input('tuning_fraction', 0.2, { type: 'number', step: 0.05 })),
    field('Stratify by', input('stratify', '')));
  modal('Import a dataset', h('div', {}, h('p', { class: 'muted', style: 'margin-top:0' }, 'Download the public datasets with scripts/fetch_datasets.sh first (see docs/instances.md).'), body),
    h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/import', formData(body))) }, 'Import'));
}
function splitModal(setPath) {
  const body = h('div', { class: 'form' }, field('Tuning fraction', input('tuning_fraction', 0.2, { type: 'number', step: 0.05 })),
    field('Stratify by factor', input('stratify', '')));
  modal(`Re-split ${setPath}`, body, h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/split', { set: setPath, ...formData(body) })) }, 'Assign splits'));
}

/* ---------------------------------------------------------------- planners */
async function planners(arg) {
  const list = await api('/api/planners');
  const open = new Set(arg ? [arg] : []);
  const wrap = h('div', {});
  const draw = () => put(wrap, h('div', { class: 'tw' }, h('table', {},
    h('thead', {}, h('tr', {}, ['Planner', 'Family', 'Adapter', 'Mode', 'Kind', 'Constraints', 'Hyperparameters', 'Validation'].map(t => h('th', {}, t)))),
    h('tbody', {}, list.flatMap(p => {
      const v = p.validation;
      const row = h('tr', { class: 'click', onclick: () => { open.has(p.name) ? open.delete(p.name) : open.add(p.name); draw(); } },
        h('td', {}, h('b', {}, p.name), p.provenance && p.provenance !== 'original' ? h('span', { class: 'badge b-warn', style: 'margin-left:6px' }, p.provenance) : null),
        h('td', {}, p.family), h('td', {}, p.adapter), h('td', {}, p.mode), h('td', {}, p.stochastic ? 'stochastic' : 'deterministic'),
        h('td', {}, p.constraints.join(', ') || '–'), h('td', {}, p.hyperparameters.length),
        h('td', {}, v ? h('span', { class: `badge ${v.passed ? 'b-ok' : 'b-bad'}` }, v.summary) : h('span', { class: 'badge b-mute' }, 'not validated')));
      if (!open.has(p.name)) return [row];
      return [row, h('tr', {}, h('td', { colspan: 8, style: 'white-space:normal;background:var(--surface-2)' },
        h('p', { style: 'margin:4px 0' }, p.description), h('p', { class: 'muted', style: 'margin:4px 0' }, p.reference),
        p.stages.length ? h('p', { style: 'margin:4px 0' }, 'Stages: ', p.stages.join(' → ')) : null,
        h('p', { style: 'margin:4px 0' }, 'Adapter parameters: ', h('code', {}, JSON.stringify(p.adapter_params))),
        table([{ label: 'Hyperparameter', key: 'name' }, { label: 'Type', key: 'kind' }, { label: 'Default', render: x => fmt(x.default) },
          { label: 'Range / choices', render: x => x.choices ? x.choices.join(', ') : `${fmt(x.low)} – ${fmt(x.high)}${x.log ? ' (log)' : ''}` },
          { label: 'Meaning', render: x => x.doc || '' }], p.hyperparameters, { empty: 'No hyperparameters.' }),
        h('div', { class: 'row', style: 'margin-top:8px' }, h('button', { onclick: e => { e.stopPropagation(); guard(() => startJob('/api/validate', { planners: [p.name] })); } }, `Validate ${p.name}`))))];
    })))));
  draw();
  put(main, h('div', { class: 'row spread' }, h('div', {}, h('h1', {}, 'Planners'),
    h('p', { class: 'sub' }, 'Registered planners and their manifests. Click a row for hyperparameters; validation runs the worked examples of each publication and, where available, compares with published results.')),
  h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/validate', {})) }, 'Validate all')), wrap);
}

/* ---------------------------------------------------------------- experiments */
async function experiments(arg) {
  const [list, plist, sets] = await Promise.all([api('/api/experiments'), api('/api/planners'), api('/api/instance-sets')]);
  const isNew = arg === '+new';
  const path = isNew ? null : (arg || (list[0] && list[0].path));
  const left = h('div', {}, h('div', { class: 'row', style: 'margin-bottom:10px' }, h('button', { class: 'primary', onclick: () => go('#/experiments/+new') }, 'New experiment')),
    list.length ? h('div', { class: 'list' }, list.map(e => h('div', { class: `item ${e.path === path ? 'sel' : ''}`, onclick: () => go(`#/experiments/${encodeURIComponent(e.path)}`) },
      h('div', { class: 't' }, e.path.split('/').pop().replace(/\.ya?ml$/, ''), e.has_results ? h('span', { class: 'badge b-info', style: 'margin-left:6px' }, 'results') : null),
      h('div', { class: 'd' }, e.error ? `⚠ ${e.error}` : `name: ${e.name} · ${(e.planners || []).length} planners · m ∈ {${(e.team_sizes || []).join(', ')}}`))))
      : h('div', { class: 'empty' }, 'No experiments yet.'));
  const editor = h('div', {});
  put(main, h('h1', {}, 'Experiments'), h('p', { class: 'sub' }, 'YAML configurations in experiments/. Check estimates the number of runs; Run and Tune start background jobs.'),
    h('div', { class: 'grid2' }, left, editor));
  if (!path && !isNew) { editor.append(h('div', { class: 'empty' }, 'Create an experiment to get started.')); return; }
  const text = isNew ? (await api('/api/experiment/template', { name: 'my_experiment' })).text : (await api('/api/experiment', { path })).text;
  const stem = isNew ? 'my_experiment' : path.split('/').pop().replace(/\.ya?ml$/, '');
  const name = input('name', stem, { style: 'width:220px' });
  const area = h('textarea', { class: 'code', spellcheck: 'false', 'aria-label': 'experiment configuration (YAML)' });
  area.value = text;
  area.addEventListener('keydown', e => { if (e.key === 'Tab') { e.preventDefault(); const s = area.selectionStart;
    area.setRangeText('  ', s, area.selectionEnd, 'end'); } });
  const msgs = h('div', { class: 'msgs' });
  const save = async () => { const r = await post('/api/experiment/save', { name: name.value, text: area.value }); toast(`Saved ${r.path}`); return r.path; };
  const check = async () => {
    const r = await post('/api/experiment/check', { text: area.value });
    put(msgs, ...r.errors.map(e => h('div', { class: 'msg-err' }, e)), ...(r.warnings || []).map(w => h('div', { class: 'msg-warn' }, w)),
      r.ok ? h('div', { class: 'msg-ok' }, `OK — ${r.instances} instances × team sizes {${r.team_sizes.join(', ')}} × ${r.start_configs.length} start configurations × ${r.planners.length} planners ≈ ${r.estimated_runs} runs (+${r.not_applicable} not applicable). Output: ${r.output}`) : null);
    return r;
  };
  const builder = h('details', { class: 'card', style: 'margin-bottom:12px' }, h('summary', { style: 'cursor:pointer;font-weight:600' }, 'Quick builder — set common fields without editing YAML'),
    h('div', { style: 'margin-top:10px' },
      h('div', { class: 'muted', style: 'font-size:12px' }, 'Instance sets'),
      h('div', {}, sets.map(st => h('label', { class: 'check' }, h('input', { type: 'checkbox', name: 'sets', value: st.path, 'data-group': 1 }), `${st.name} (${st.count})`)), sets.length ? null : h('span', { class: 'muted' }, 'none — generate or import first')),
      h('div', { class: 'muted', style: 'font-size:12px;margin-top:8px' }, 'Planners'),
      h('div', {}, plist.map(p => h('label', { class: 'check' }, h('input', { type: 'checkbox', name: 'planners', value: p.name, 'data-group': 1, checked: true }), p.name))),
      h('div', { class: 'form', style: 'margin-top:10px' },
        field('Team sizes', input('team_sizes', '1 2 4')), field('Start configurations', input('start_configs', 'dispersed')),
        field('Split', select('split', ['test', 'tuning', 'all'], 'test')), field('Seeds (stochastic)', input('seeds', 5, { type: 'number' })),
        field('Timing repetitions', input('repetitions', 3, { type: 'number' })), field('Workers', input('workers', 4, { type: 'number' })),
        field('Time limit [s]', input('time_limit', 60, { type: 'number' })), field('Tolerance τ', input('tolerance', 0.02, { type: 'number', step: 0.005 })),
        field('Profiles', input('profiles', 'core')), field('Configurations', input('configs', 'default', { placeholder: 'default tuned' }))),
      h('div', { class: 'row', style: 'margin-top:10px' }, h('button', { onclick: () => guard(async () => {
        const r = await post('/api/experiment/compose', { text: area.value, fields: formData(builder) }); area.value = r.text; toast('Applied to YAML'); }) }, 'Apply to YAML'))));
  put(editor, h('div', { class: 'row spread', style: 'margin-bottom:10px' }, h('div', { class: 'row' }, 'File experiments/', name, '.yaml'),
    h('div', { class: 'row' },
      h('button', { onclick: () => guard(async () => { const p = await save(); go(`#/experiments/${encodeURIComponent(p)}`); }) }, 'Save'),
      h('button', { onclick: () => guard(check) }, 'Check'),
      h('button', { onclick: () => guard(async () => { const p = await save(); tuneModal(p); }) }, 'Tune…'),
      h('button', { class: 'primary', onclick: () => guard(async () => { const r = await check(); if (!r.ok) return toast('Fix the errors first', true);
        const p = await save(); runModal(p, r); }) }, 'Run…'))),
  msgs, builder, area);
}
function runModal(config, est) {
  const body = h('div', {}, h('p', { style: 'margin-top:0' }, `About ${est.estimated_runs} runs will be executed; results go to ${est.output}.`),
    h('div', { class: 'form' }, field('Worker processes', input('workers', '', { type: 'number', min: 1, placeholder: 'from config' }))),
    h('label', { class: 'check', style: 'margin-top:10px' }, h('input', { type: 'checkbox', name: 'resume', checked: true }), 'Resume: skip runs already recorded'));
  modal(`Run ${config}`, body, h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/run', { config, ...formData(body) })) }, 'Start run'));
}
function tuneModal(config) {
  const body = h('div', {}, h('p', { class: 'muted', style: 'margin-top:0' }, 'Bayesian optimisation on the tuning split with the same budget for every planner; the best configuration is frozen to the tuning directory.'),
    h('div', { class: 'form' }, field('Budget (evaluations)', input('budget', 20, { type: 'number', min: 1 })),
      field('Seeds per evaluation (stochastic)', input('tuning_seeds', 3, { type: 'number', min: 1 })),
      field('Workers', input('workers', '', { type: 'number', placeholder: 'from config' }))));
  modal(`Tune ${config}`, body, h('button', { class: 'primary', onclick: () => guard(() => startJob('/api/tune', { config, ...formData(body) })) }, 'Start tuning'));
}

/* ---------------------------------------------------------------- jobs */
async function jobs(arg) {
  const list = await api('/api/jobs');
  const id = arg || (list[0] && list[0].id);
  const detail = h('div', {});
  put(main, h('div', { class: 'row spread' }, h('div', {}, h('h1', {}, 'Jobs'),
    h('p', { class: 'sub' }, 'Background commands started from the TUI or the web UI. They keep running when the interface is closed.')),
  h('button', { onclick: () => guard(async () => { const r = await post('/api/jobs/clear', {}); toast(`Removed ${r.removed} finished jobs`); render(); }) }, 'Clear finished')),
  jobsTable(list, id), h('div', { style: 'height:16px' }), detail);
  if (!id) return;
  const pre = h('pre', { class: 'log' });
  const head = h('div', {});
  let follow = true;
  pre.addEventListener('scroll', () => { follow = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 20; });
  const update = async () => {
    const j = await api('/api/job', { id });
    put(head, h('div', { class: 'row spread' }, h('div', { class: 'row' }, h('h2', { style: 'margin:0' }, j.title), badge(j.state),
      j.progress ? h('div', { class: 'row' }, bar(j.progress.fraction), `${j.progress.done}/${j.progress.total}`) : null, h('span', { class: 'muted' }, dur(j.elapsed))),
    h('div', { class: 'row' },
      ...j.artifacts.map(a => h('a', { class: 'btn', href: fileUrl(a.path), target: '_blank', rel: 'noopener' }, `Open ${a.kind}`)),
      ['queued', 'running'].includes(j.state) ? h('button', { class: 'danger', onclick: () => guard(async () => { await post('/api/job/cancel', { id }); render(); }) }, 'Cancel')
        : h('button', { onclick: () => guard(async () => { await post('/api/job/remove', { id }); go('#/jobs'); }) }, 'Remove'))),
    h('p', { class: 'muted', style: 'margin:6px 0 10px' }, h('code', {}, `uninbench ${j.argv.join(' ')}`), j.returncode !== null && j.returncode !== undefined ? ` · exit code ${j.returncode}` : ''));
    pre.textContent = j.log || '(no output yet)';
    if (follow) pre.scrollTop = pre.scrollHeight;
    const active = ['queued', 'running'].includes(j.state);
    if (!active) { clearInterval(pollTimer); refreshNav(); if (wasActive) setTimeout(render, 300); }
    wasActive = active;
    return j;
  };
  let wasActive = false;
  put(detail, h('div', { class: 'card' }, head, pre));
  const j = await update();
  const listStale = list.some(x => x.id === id && x.state !== j.state);
  if (['queued', 'running'].includes(j.state)) pollTimer = setInterval(() => update().catch(() => {}), 1200);
  else if (listStale) setTimeout(render, 300);
}

/* ---------------------------------------------------------------- results */
async function results(arg) {
  const list = await api('/api/results');
  const dir = arg || (list[0] && list[0].path);
  const left = list.length ? h('div', { class: 'list' }, list.map(r => h('div', { class: `item ${r.path === dir ? 'sel' : ''}`, onclick: () => go(`#/results/${encodeURIComponent(r.path)}`) },
    h('div', { class: 't' }, r.name), h('div', { class: 'd' }, `${r.runs} runs · ${ago(r.modified)}`), h('div', { style: 'margin-top:6px' }, stack(r.outcomes)))))
    : h('div', { class: 'empty' }, 'No results yet. Run an experiment first.');
  const right = h('div', {});
  put(main, h('h1', {}, 'Results'), h('p', { class: 'sub' }, 'Experiment outputs in results/: outcome summaries, statistical reports and replays.'), h('div', { class: 'grid2' }, left, right));
  if (!dir) return;
  const s = await api('/api/results/summary', { dir });
  const inst = select('instance', s.replay_instances, s.replay_instances[0]);
  const runsBox = h('div', {});
  const planners = [...new Set(s.success.map(r => r.planner.replace(/\*$/, '').replace(/@.*$/, '')))];
  const pf = select('planner', [['', 'all planners'], ...planners.map(p => [p, p])], '');
  const of = select('outcome', [['', 'all outcomes'], ...Object.keys(s.outcomes).map(o => [o, o])], '');
  const loadRuns = async () => {
    const rows = await api('/api/runs', { dir, planner: pf.value, outcome: of.value, limit: 300 });
    put(runsBox, table([{ label: 'Planner', key: 'variant' }, { label: 'Instance', key: 'instance_id' }, { label: 'm', key: 'team_size' },
      { label: 'Config', key: 'config_label' }, { label: 'Seed', key: 'seed' }, { label: 'Outcome', render: r => badge(r.outcome) },
      { label: 'Coverage', render: r => fmt(r.coverage_ratio) }, { label: 'T/T_LB', render: r => fmt(r.normalized_makespan) },
      { label: 'Overlap', render: r => fmt(r.overlap_ratio) }, { label: 'Plan time [s]', render: r => fmt(r.planning_time) },
      { label: 'Reason', render: r => r.reason ? String(r.reason).slice(0, 60) : '' }], rows, { empty: 'No runs match.' }));
  };
  pf.addEventListener('change', loadRuns); of.addEventListener('change', loadRuns);
  const prim = s.descriptives.filter(r => r.metric === 'normalized_makespan');
  put(right, 
    h('div', { class: 'row spread' }, h('h2', { style: 'margin:0' }, s.name), h('div', { class: 'row' },
      s.report ? h('a', { class: 'btn primary', href: fileUrl(s.report), target: '_blank', rel: 'noopener' }, 'Open report') : null,
      h('button', { onclick: () => guard(() => startJob('/api/report', { dir })) }, s.report ? 'Rebuild report' : 'Build report'))),
    h('div', { class: 'kpis', style: 'margin-top:12px' }, Object.entries(s.outcomes).map(([k, v]) => h('div', { class: 'kpi' }, h('b', {}, v), h('span', {}, k.replace('_', ' '))))),
    h('div', { class: 'card' }, h('h3', { style: 'margin-top:0' }, 'Replay'),
      s.replay_instances.length ? h('div', { class: 'row' }, inst, h('button', { onclick: () => guard(() => startJob('/api/replay', { dir, instance: inst.value })) }, 'Build replay')) : h('div', { class: 'muted' }, 'No stored trajectories.'),
      s.replays.length ? h('div', { class: 'row', style: 'margin-top:8px' }, s.replays.map(p => h('a', { class: 'btn', href: fileUrl(p), target: '_blank', rel: 'noopener' }, p.split('/').pop().replace('.html', '')))) : null),
    h('h2', {}, 'Success rates'),
    table([{ label: 'Family', key: 'family' }, { label: 'm', key: 'team_size' }, { label: 'Config', key: 'config' }, { label: 'Planner', key: 'planner' },
      { label: 'Runs', key: 'runs' }, { label: 'Success rate', render: r => r.success_rate === null ? '–' : h('div', { class: 'row' }, bar(r.success_rate), fmt(r.success_rate)) },
      { label: 'Incomplete', key: 'incomplete' }, { label: 'Invalid', key: 'invalid' }, { label: 'Timeout', key: 'timeout' }, { label: 'Crash', key: 'crash' }, { label: 'n/a', key: 'not_applicable' }], s.success),
    h('h2', {}, 'Normalized makespan T/T_LB (successful runs)'),
    table([{ label: 'Family', key: 'family' }, { label: 'm', key: 'team_size' }, { label: 'Planner', key: 'planner' }, { label: 'Instances', key: 'n' },
      { label: 'Median', render: r => fmt(r.median) }, { label: 'IQR', render: r => `${fmt(r.q1)} – ${fmt(r.q3)}` }, { label: '95 % CI', render: r => `${fmt(r.ci_lo)} – ${fmt(r.ci_hi)}` }], prim,
    { empty: 'No successful runs yet.' }),
    h('div', { class: 'row spread' }, h('h2', {}, 'Runs'), h('div', { class: 'row' }, pf, of)), runsBox);
  loadRuns();
}

/* ---------------------------------------------------------------- start */
refreshNav();
navTimer = setInterval(refreshNav, 4000);
render();
