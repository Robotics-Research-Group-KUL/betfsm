
// script.js

const svg = d3.select("#tree");
const activeIds = new Set();
let replayFrames = [], replayIndex = 0, playing = false;
let isTransitioning = false;


async function loadTree() {
    const res = await fetch("/api/tree");
    const data = await res.json();
    const nodes = data.nodes.map(node => ({
        id: node.id,
        name: node.name,
        type: node.type,
        parentId: node.parentId
    }));
    const stratify = d3.stratify().id(d => d.id).parentId(d => d.parentId);
    const root = stratify(nodes);
    window.root = root;
    renderTree(root);
}

// Collapse/expand children
function toggleChildren(d) {
    console.log("toggleChildren called")
    if (d.children) {
        d._children = d.children;
        d.children = null;
    } else if (d._children) {
        d.children = d._children;
        d._children = null;
    }
}

// Collapse every node that has children
function collapseAll(root) {
    root.each(d => {
        if (d.children) {
            d._children = d.children;
            d.children = null;
        }
    });
}

// Expand every node that has hidden children
function expandAll(root) {
    root.each(d => {
        if (d._children) {
            d.children = d._children;
            d._children = null;
        }
    });
}

function expandActiveBranches(root) {
    root.each(d => {
        if (d._children && hasActiveDescendant(d)) {
            d.children = d._children;
            d._children = null;
        } else if (d.children && !hasActiveDescendant(d)) {
            d._children = d.children;
            d.children = null;
        }
    });
}

function hasActiveDescendant(node) {
    if (activeIds.has(node.data.id)) return true;
    if (node.children) {
        return node.children.some(child => hasActiveDescendant(child));
    }
    if (node._children) {
        return node._children.some(child => hasActiveDescendant(child));
    }
    return false;
}

// Collect only visible descendants
function getVisibleDescendants(root) {
    const nodes = [];
    function recurse(node) {
        nodes.push(node);
        if (node.children) {
            node.children.forEach(recurse);
        }
    }
    recurse(root);
    return nodes;
}

// Collect only visible links
function getVisibleLinks(root) {
    const links = [];
    function recurse(node) {
        if (node.children) {
            node.children.forEach(child => {
                links.push({ source: node, target: child });
                recurse(child);
            });
        }
    }
    recurse(root);
    return links;
}

//function assignVisibleDepths(root) {
//    let maxDepth = 0;
//    function recurse(node, depth) {
//        node.visibleDepth = depth;
//        maxDepth = Math.max(maxDepth, depth);
//        if (node.children) {
//            node.children.forEach(child => recurse(child, depth + 1));
//        }
//    }
//    recurse(root, 0);
//    return maxDepth;
//}
//

/**
 * Recompute depth and height for a d3.stratify() result.
 * @param {Object} root - The root node of the hierarchy.
 */
function recomputeDepthAndHeight(root) {
  // First, recompute depth with a DFS
  function setDepth(node, depth) {
    node.depth = depth;
    if (node.children) {
      node.children.forEach(child => setDepth(child, depth + 1));
    }
  }

  // Then, recompute height bottom-up
  function setHeight(node) {
    if (!node.children || node.children.length === 0) {
      node.height = 0;
    } else {
      node.height = 1 + Math.max(...node.children.map(setHeight));
    }
    return node.height;
  }

  setDepth(root, 0);
  setHeight(root);
}

function renderTree(root) {
    console.log("renderTree called");

    const NODE_WIDTH = 200;
    const NODE_HEIGHT = 46;

    const H_SPACING = NODE_WIDTH + 60;
    const V_SPACING = NODE_HEIGHT + 20;

    const TOP_PADDING = 40;
    const LEFT_PADDING = 20;

    const treeLayout = d3.tree().nodeSize([V_SPACING, H_SPACING]);
    treeLayout(root);

    recomputeDepthAndHeight(root);

    const nodes = getVisibleDescendants(root);
    const minX = d3.min(nodes, d => d.x);
    const maxX = d3.max(nodes, d => d.x);
    const minY = d3.min(nodes, d => d.y);
    const maxY = d3.max(nodes, d => d.y);

    root.each(d => {
        d.x = d.x - minX + TOP_PADDING;
        d.y = d.y - minY + LEFT_PADDING;
    });

    svg.attr("height", maxX - minX + NODE_HEIGHT + TOP_PADDING + 50);
    svg.attr("width", maxY - minY + NODE_WIDTH + LEFT_PADDING + 50);

    // ----------------------------------------------------------------------
    // LINKS
    // ----------------------------------------------------------------------
    //const link = svg.selectAll(".link")
    //    .data(getVisibleLinks(root), d => d.target.id);

    const link = svg.selectAll(".link")
        .data(getVisibleLinks(root), d => `${d.source.id}-${d.target.id}`);

    link.join(
        enter => enter.append("path")
            .attr("class", "link")
            .attr("d", d3.linkHorizontal()
                .source(d => [d.source.y + NODE_WIDTH, d.source.x + NODE_HEIGHT / 2])
                .target(d => [d.source.y + NODE_WIDTH, d.source.x + NODE_HEIGHT / 2]) // start collapsed
            )
            .transition()
                .duration(500)
                .on("start", () => {isTransitioning=true;})
                .on("end", () => {isTransitioning=false;})
                .ease(d3.easeCubic)
                .attr("d", d3.linkHorizontal()
                .source(d => [d.source.y + NODE_WIDTH, d.source.x + NODE_HEIGHT / 2])
                .target(d => [d.target.y, d.target.x + NODE_HEIGHT / 2])),
        update => update.transition()
            .duration(500)
            .on("start", () => {isTransitioning=true;})
            .on("end", () => {isTransitioning=false;})
            .ease(d3.easeCubic)
            .attr("d", d3.linkHorizontal()
                .source(d => [d.source.y + NODE_WIDTH, d.source.x + NODE_HEIGHT / 2])
                .target(d => [d.target.y, d.target.x + NODE_HEIGHT / 2])),
        exit => exit.transition().duration(500).style("opacity", 0).remove()
    );

    // ----------------------------------------------------------------------
    // NODES
    // ----------------------------------------------------------------------
    const nodesSelection = svg.selectAll("g.node")
        .data(getVisibleDescendants(root), d => d.id);

    const nodesEnter = nodesSelection.enter()
        .append("g")
        .attr("class", "node")
        .attr("transform", d => `translate(${d.parent ? d.parent.y : d.y},${d.parent ? d.parent.x : d.x})`) // start at parent
        .style("opacity", 0)
        .on("click", handleNodeClick);

    nodesEnter.append("rect")
        .attr("rx", 8).attr("ry", 8)
        .attr("width", NODE_WIDTH).attr("height", NODE_HEIGHT)
        .attr("class", "node-rect")
        .style("cursor", "pointer")
        .style("pointer-events", "all");

    nodesEnter.append("text")
        .attr("dy", "2.5em")
        .attr("x", NODE_WIDTH / 2)
        .attr("text-anchor", "middle")
        .style("pointer-events", "none")
        .text(d => d.data.name);

    nodesEnter.append("text")
        .attr("dy", "1.1em")
        .attr("x", NODE_WIDTH / 2)
        .attr("text-anchor", "middle")
        .style("pointer-events", "none")
        .text(d => `<${d.data.type}>`);

    nodesEnter.append("text")
        .attr("class", "toggle-marker")
        .attr("x", NODE_WIDTH - 10)
        .attr("y", NODE_HEIGHT / 2)
        .attr("text-anchor", "middle")
        .attr("alignment-baseline", "middle")
        .style("pointer-events", "none")
        .text(d => d._children ? "+" : d.children ? "--" : "");

    // Animate entering nodes to their position
    nodesEnter.transition()
        .duration(500)
        .on("start", () => { isTransitioning = true; }) 
        .on("end", () => { isTransitioning = false; })
        .ease(d3.easeCubic)
        .attr("transform", d => `translate(${d.y},${d.x})`)
        .style("opacity", 1);

    // Animate updates
    nodesSelection.transition().duration(500).ease(d3.easeCubic)
        .attr("transform", d => `translate(${d.y},${d.x})`);

    // Update toggle markers
    svg.selectAll(".toggle-marker")
        .text(d => d._children ? "+" : d.children ? "--" : "");

    // Animate exit
    nodesSelection.exit()
        .transition().duration(500).ease(d3.easeCubic)
        .style("opacity", 0)
        .remove();

    svg.selectAll(".node-rect")
        .classed("collapsed", d => d._children);
}
function applyActivity() {
    svg.selectAll(".node-rect")
        .classed("active", d => activeIds.has(d.data.id))
        .classed("inactive", d => !activeIds.has(d.data.id));
}

function handleTick(msg) {
    if (isTransitioning) {
        return;
    }
    activeIds.clear();
    if (msg.active) {
        msg.active.forEach(id => activeIds.add(id));
    }
    if (autoExpand) {
        expandActiveBranches(window.root);
        renderTree(window.root);
    }
    applyActivity();
}
function handleNodeClick(event, d) {
    console.log("handleNodeClick")
    console.log(d)
    toggleChildren(d);
    console.log(d)
    renderTree(window.root);
    applyActivity();
}

// WebSocket
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
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

let autoExpand = false;
document.getElementById("autoExpand").onchange = ev => {
  autoExpand = ev.target.checked;
};

document.getElementById("collapseAll").onclick = () => {
    collapseAll(window.root);
    renderTree(window.root);
    applyActivity();
};

document.getElementById("expandAll").onclick = () => {
    expandAll(window.root);
    renderTree(window.root);
    applyActivity();
};
loadTree();
