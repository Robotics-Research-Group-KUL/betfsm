<div style="position:relative; width:100%; background:#004070;">
  <div style="max-width:1200px; margin:0 auto; padding:16px; display:flex; align-items:center; gap:24px;">
    <img src="./logo.png" alt="BeTFSM logo" style="height:86px; width:auto; display:block;" />
    <div style="color:#ffffff;">
      <div style="font-size:50px; font-weight:700; line-height:1.2;"> &nbsp; </div>
      <div style="font-size:16px; font-weight:500; opacity:0.9;">Behavior Trees &amp; Finite State Machines for Robotics</div>
    </div>
  </div>
</div>

<p/>

BeTFSM is a library for "ticking" statemachines and behavior trees. In this
unified framework, discrete tasks can be specified with the modularity of
behavior trees and at the lower-level precise interaction can be specified
using state machines. It targets discrete coordination of robotic systems at
both high- and low level.

<p/>

Published under the GNU LESSER GENERAL PUBLIC LICENSE Version 3, 29 June 2007.

(c) 2024, KU Leuven, Department of Mechanical Engineering, ROB-Group: Erwin Aertbeliën, 
contributions of Federico Ulloa Rios and Santiago Iregui Rincon

<p>
  <a href="https://aiprism.eu/"><img src="./Ai-Prism_Logo_Horizontal.png" alt="AI-PRISM Logo" width="250" align="left" style="margin-right:35px;"/></a>
  This work was funded by the European Union’s Horizon 2020 research and innovation program 
  under <a href="https://aiprism.eu/">grant agreement No. 101058589 AI-Prism</a>
</p>




## Documentation

[Gitlab pages documentation](https://betfsm-90d316.pages.gitlab.kuleuven.be/)


## Generate most recent documentation (for developers)

Documentation provided in mkdocs, with the library build and installed, you
can access documentation by:

```
cd ./documentation
./browse
```
This will generate a python environment to run mkdocs with the appropriate libraries.
In order for the python documentation to be generated, the libraries have to be installed and build.


You can build the documentation into a directory `./site' using:
```
cd ./documentation
venv/bin/mkdocs build
```

## Used documentation:
[ros2-tutorial](https://ros2-tutorial.readthedocs.io/en/latest/python_node_explained.html)

[ros2 github examples](https://github.com/ros2/exampleshttps://github.com/ros2/examples)
[demos](https://github.com/ros2/demos/tree/ea3661152a87bc48e3f277ca8131c85a1a23d661)

[mkdocstrings](https://mkdocstrings.github.io/griffe/reference/docstrings/)


[paper](https://arxiv.org/pdf/2208.04211)


