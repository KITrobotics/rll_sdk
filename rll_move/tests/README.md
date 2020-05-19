# `rll_move` tests

## Build the package

```bash
catkin build rll_move
```


## Run tests with `catkin`

```bash
catkin run_tests rll_move
```

will build the whole package and run the tests registered in the `CMakeLists.txt`.


To inspect the results, run:

```bash
catkin_test_results ~/rll_ws/build/rll_move --all
```

**Hint:** If the tests seem stuck make sure you have no other `move_iface` instances running. Usually, this shouldn't be a problem since tests start their own ros_core (if required) and run in a separate test setup.


## Build only the tests

If you only want to build your C++ tests:

```bash
catkin build rll_move --make-args tests
```


## Run a single test locally

Make sure to build your C++ tests before running them.

Run a test launch file locally:

```bash
rostest rll_move unit_tests_cpp.test
```

the available tests so far are:


```bash
rostest rll_move movement_tests.test
rostest rll_move unit_tests_cpp.test
```


If you want to see the test output in the terminal run it with `--text`, e.g.:

```bash
rostest rll_move movement_tests.test --text
```

you can also view the rviz visualisation of your test by passing additional parameters to the test (launch) file as usual:

```bash
rostest rll_move movement_tests.test --text headless:=false
```


## Writing tests

See the [ROS Wiki](http://wiki.ros.org/rostest/Writing) for detailed information.

### 1. Create a `.test` file

Create a .test file, which is just a regular launch file, to start the required nodes. Depending on your tests you need to start a `move_iface` before your tests can be run. This isn't required for basic unit tests. Have a look at the examples to get started.

#### Examples:

The `launch/movement_tests.test` starts a (headless) `move_iface` and runs python tests which interact with the `move_iface`.
The `movement_tests_main.py` file contains the list of actual test cases to run and for the beginning you can simply add your own tests to this list.

The `launch/unit_tests_cpp.test` runs only C++ unit tests and therefore doesn't start a `move_iface` at all.


### 2. Create a test case

Have a look at the `basic_movements.py` in the `scripts` folder. By inheriting from `TestCaseWithRLLMoveClient` you have a standard [`unittest.TestCase`](https://docs.python.org/3/library/unittest.html) to work with and can access a [`MoveClient`](https://rll-doc.ipr.kit.edu/rll_move_client.html) instance with `self.client` to interact with the `move_iface`. The test case defined in this file is included in `movement_tests_main.py`.

There are also some predefined assertion functions like `assert_last_srv_call_success` to help you to validate service call results.