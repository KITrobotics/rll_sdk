# `rll_move` tests

## Build and run all tests with `catkin`

```bash
catkin run_tests rll_move
```
If the tests seem stuck you might need to have a roscore running.

## Run a single test locally

Make sure to build your C++ tests before running them.

Run a test launch file locally:

```bash
rostest rll_move move_iface_unit.test
```

If you want to see the output run it with `--text`, e.g.:

```bash
rostest rll_move move_iface_unit.test --text
```
