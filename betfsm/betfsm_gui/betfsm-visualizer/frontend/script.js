
const svg = d3.select("#tree");
const activeIds = new Set();
let replayFrames = [], replayIndex = 0, playing = false;

async function loadTree() {
    console.log("loadTree() entered")
    try{
      const res = await fetch("/api/tree");
      const data = await res.json();
      window.data = data
      console.log(data)
      renderTree(data);
    } catch (error) {
        console.log("Error occured")
        console.log(error)
    }
    console.log("loadTree() exit")
}

function toggleChildren(d) {
    if (d.children) {
        // If children exist, move them to _children (private property)
        d._children = d.children;
        d.children = null;
    } else if (d._children) {
        // If _children exist, move them back to children
        d.children = d._children;
        d._children = null;
    }
}
function renderTree(data) {
  console.log("renderTree() entered")
  // Convert the backend format to a format that d3.stratify can understand
  const nodes = data.nodes.map(node => ({
    id: node.id,
    name: node.name,
    type: node.type,
    parentId: node.parentId
  }));
  
  const stratify = d3.stratify()
  const root = stratify(nodes);
  const treeLayout = d3.tree().size([800, 1000]);
  treeLayout(root);

  const NODE_WIDTH = 200; // Defined by nodesSelection.append("rect").attr("width", 140)
  const NODE_HEIGHT = 46; // Defined by nodesSelection.append("rect").attr("height", 46)

//
//  // ----------------------------------------------------------------------
//  // 1. UPDATE LINKS
//  // ----------------------------------------------------------------------
//  const link = svg.selectAll(".link")
//    .data(root.links(), d => d.target.id)
//    .join(
//      enter => enter.append("path")
//        .attr("class", "link")
//        .attr("fill", "none")
//        .attr("stroke", "#ccc")
//        .attr("stroke-width", 1.5)
//        // Set initial position for new links at the parent's starting point
//        .attr("d", d => {
//            const o = {x: d.source.x, y: d.source.y + NODE_WIDTH};
//            return d3.linkHorizontal().source(o).target(o)(d);
//        }),
//      update => update.transition().duration(duration)
//        .attr("d", d3.linkHorizontal()
//          .source(d => [d.source.y + NODE_WIDTH, d.source.x])
//          .target(d => [d.target.y, d.target.x])),
//      exit => exit.transition().duration(duration).remove()
//        .attr("d", d => {
//            // Transition exiting links back to the parent's position
//            const o = {x: d.source.x, y: d.source.y + NODE_WIDTH};
//            return d3.linkHorizontal().source(o).target(o)(d);
//        })
//    );
//  
//  // ----------------------------------------------------------------------
//  // 2. UPDATE NODES
//  // ----------------------------------------------------------------------
//  const nodesSelection = svg.selectAll("g.node")
//    // Use descendants to bind data
//    .data(descendants, d => d.id)
//    .join(
//      enter => {
//        const g = enter.append("g")
//          .attr("class", "node")
//          // Set initial position of new nodes at the parent's location (for transition)
//          .attr("transform", d => {
//              const parent = d.parent || d; // Use self if no parent
//              return `translate(${parent.y},${parent.x})`;
//          })
//          // ADD THE CLICK HANDLER HERE
//          .on("click", handleNodeClick);
//
//        g.append("rect")
//          .attr("rx", 8).attr("ry", 8)
//          .attr("width", NODE_WIDTH).attr("height", NODE_HEIGHT)
//          .attr("class", "node-rect");
//
//        // ... (Append Name and Type text elements as you have them, unchanged) ...
//        g.append("text")
//          .attr("dy", "2.5em")
//          .attr("x", NODE_WIDTH/2)
//          .attr("text-anchor", "middle")
//          .text(d => d.data.name);
//
//        g.append("text")
//          .attr("dy", "1.1em")
//          .attr("x", NODE_WIDTH/2)
//          .attr("text-anchor", "middle")
//          .text(d => `<${d.data.type}>`);
//          
//        return g;
//      },
//      update => update, // Pass update selection through
//      exit => exit.transition().duration(duration).remove()
//        .attr("transform", d => {
//            // Transition exiting nodes back to the parent's location
//            const parent = d.parent || d;
//            return `translate(${parent.y},${parent.x})`;
//        })
//    );
//
//  // Transition all nodes to their new positions
//  nodesSelection.transition().duration(duration)
//    .attr("transform", d => `translate(${d.y},${d.x})`);
//
//  // Update the class to show if a node has collapsed children
//  nodesSelection.select(".node-rect")
//    .classed("collapsed", d => d._children);
//}
//



  // 1. ADD LINKS
  const link = svg.selectAll(".link")
    .data(root.links(), d => d.target.id)
    .join("path")
    .attr("class", "link")
    .attr("fill", "none")
    .attr("stroke", "#ccc")
    .attr("stroke-width", 1.5)
    .attr("d", d3.linkHorizontal()
      // The parent node is the 'source'
      .source(d => [d.source.y + NODE_WIDTH, d.source.x+NODE_HEIGHT/2]) // Start from the right edge of the parent box (y + 140)

      // The child node is the 'target'
      .target(d => [d.target.y, d.target.x+NODE_HEIGHT/2])); // End at the left edge of the child box (y)
  const nodesSelection = svg.selectAll("g.node")
    .data(root.descendants(), d => d.id)
    .join("g")
    .attr("class", "node")
    .attr("transform", d => `translate(${d.y},${d.x})`);

  nodesSelection.append("rect")
    .attr("rx", 8).attr("ry", 8)
    .attr("width", NODE_WIDTH).attr("height", NODE_HEIGHT)
    .attr("class", "node-rect");

  nodesSelection.append("text")
    // Keep name on the first line (default vertical position is fine)
    .attr("dy", "2.5em")
    .attr("x", NODE_WIDTH/2) // Center the text horizontally (half of the 140 width)
    .attr("text-anchor", "middle")
    .text(d => d.data.name);

  nodesSelection.append("text")
    // Move type to the second line
    .attr("dy", "1.1em")
    .attr("x", NODE_WIDTH/2) // Center the text horizontally
    .attr("text-anchor", "middle")
    .text(d => `<${d.data.type}>`); // <--- CHANGE IS HERE!
}

function applyActivity() {
  svg.selectAll(".node-rect")
    .classed("active", d => activeIds.has(d.data.id))
    .classed("inactive", d => !activeIds.has(d.data.id));
}

function handleTick(msg) {
  activeIds.clear();
  if (msg.active) {
    msg.active.forEach(id => activeIds.add(id));
  }
  applyActivity();
}

function handleNodeClick(event, d) {
    toggleChildren(d);
    renderTree(window.data);
    applyActivity(); // Re-apply activity class after re-rendering
}

// Adjust WebSocket URL to match the backend endpoint
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
console.log(`${wsProtocol}//${location.host}/ws/stream`);
const ws = new WebSocket(`${wsProtocol}//${location.host}/ws/stream`);
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
