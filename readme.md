# Yasmin_etasl


Documentation provided in mkdocs, with the library build and installed, you
can access documentation by:

```
cd \documentation
./browse
```
This will generate a python environment to run mkdocs with the appropriate libraries.
In order for the python documentation to be generated, the libraries have to be installed.


## Used documentation:
[ros2-tutorial](https://ros2-tutorial.readthedocs.io/en/latest/python_node_explained.html)

[ros2 github examples](https://github.com/ros2/exampleshttps://github.com/ros2/examples)
[demos](https://github.com/ros2/demos/tree/ea3661152a87bc48e3f277ca8131c85a1a23d661)

[mkdocstrings](https://mkdocstrings.github.io/griffe/reference/docstrings/)





### Some testing of graphviz format:

- The newline should be double escapred
- fillcolor=lightblue for ENTRY/EXIT, fillcolor= "#ff634f" for DOO.


```graphviz

digraph G {
    rankdir=LR
    base[label="", shape=point];
    node [shape=rectangle style="filled,rounded" fillcolor=lightblue ];
  base -> base_concurrent_1;
  node [shape=rectangle style="filled,rounded" fillcolor="#51b6e4" label="concurrent\n<ConcurrentSequence> ];
    base_concurrent_1 -> base_concurrent_1_task1_1;
    node [shape=rectangle style="filled,rounded" fillcolor="#51b6e4" label="task1\n<Sequence> ];
      base_concurrent_1_task1_1 -> base_concurrent_1_task1_1_waiting_1;
      node [shape=rectangle style="filled,rounded" fillcolor="#51b6e4" label="waiting\n<TimedWait> ];
      base_concurrent_1_task1_1 -> base_concurrent_1_task1_1_message_2;
      node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="message\n<Message> ];
    base_concurrent_1 -> base_concurrent_1_my_state_machine_2;
    node [shape=rectangle style="filled,rounded" fillcolor="#51b6e4" label="my_state_machine\n<MyStateMachine> ];
      base_concurrent_1_my_state_machine_2 -> base_concurrent_1_my_state_machine_2_A_1;
      node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="A\n<TimedWait> ];
      base_concurrent_1_my_state_machine_2 -> base_concurrent_1_my_state_machine_2_B_2;
      node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="B\n<Sequence> ];
        base_concurrent_1_my_state_machine_2_B_2 -> base_concurrent_1_my_state_machine_2_B_2_a_1;
        node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="a\n<TimedWait> ];
        base_concurrent_1_my_state_machine_2_B_2 -> base_concurrent_1_my_state_machine_2_B_2_b_2;
        node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="b\n<TimedWait> ];
        base_concurrent_1_my_state_machine_2_B_2 -> base_concurrent_1_my_state_machine_2_B_2_c_3;
        node [shape=rectangle style="filled,rounded" fillcolor="##003e76" label="c\n<TimedWait> ];
      base_concurrent_1_my_state_machine_2 -> base_concurrent_1_my_state_machine_2_C_3;
      node [shape=rectangle style="filled,rounded" fillcolor="#51b6e4" label="C\n<TimedWait> ];

}

}

```