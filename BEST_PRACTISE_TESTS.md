* If an event is expected to take some time, it is better to check and retry multiple times instead of waiting once an checking then.
* When using `PilzModbusServerMock`, use a distinct port for every instance that *may* be run in short succession
* Use rostest only if you explicitly want to test some ROS behaviour or communication method. Otherwise gtest is faster.
