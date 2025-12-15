
const svg = d3.select("#tree");
const activeIds = new Set();
let replayFrames = [], replayIndex = 0, playing = false;

async function loadTree() {
  const res = await fetch("/api/tree");
  const data = await res.json();
  renderTree(data);
}

function renderTree(data) {
  const stratify = d3.stratify()
    .id(d => d.id)
    .parentId(d => findParentId(data, d.id));
  const root = stratify(data.nodes);
  const treeLayout = d3.tree().size([800, 1000]);
  treeLayout(root);

  const nodes = svg.selectAll("g.node")
    .data(root.descendants(), d => d.id)
    .join("g")
    .attr("class", "node")
    .attr("transform", d => `translate(${d.y},${d.x})`);

  nodes.append("rect")
    .attr("rx", 8).attr("ry", 8)
    .attr("width", 140).attr("height", 46)
    .attr("class", "node-rect");

  nodes.append("text").attr("dy", "-0.2em").text(d => d.data.name);
  nodes.append("text").attr("dy", "1.1em").text(d => d.data.type);
}

function findParentId(data, id) {
  const node = data.nodes.find(n => n.id === id);
  for (const n of data.nodes) {
    if (n.children.includes(id)) return n.id;
  }
  return null;
}

function applyActivity() {
  svg.selectAll(".node-rect")
    .classed("active", d => activeIds.has(d.data.id))
    .classed("inactive", d => !activeIds.has(d.data.id));
}

function handleTick(msg) {
  activeIds.clear();
  msg.active.forEach(id => activeIds.add(id));
  applyActivity();
}

const ws = new WebSocket(`ws://${location.host}/ws/stream`);
ws.onmessage = ev => handleTick(JSON.parse(ev.data));

document.getElementById("play").onclick = () => {
  playing = !playing;
  if (playing) stepReplay();
};

function stepReplay() {
  if (!playing) return;
  const frame = replayFrames[replayIndex];
  activeIds.clear();
  frame.active.forEach(id => activeIds.add(id));
  applyActivity();
  replayIndex = (replayIndex + 1) % replayFrames.length;
  setTimeout(stepReplay, 33);
}

loadTree();
