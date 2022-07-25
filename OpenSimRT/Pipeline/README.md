# Pipeline

## Design pattern: the quasi-ros topic

Each node starts with a pipe which both reads and writes CommonStamped messages. These are the SimTk tables, which also need a header. In order to not have too many custom messages and not need to pass the header with each message, which would increase message overhead, a service was created to pass the input headers. 

As such, each output also is coupled with a service server which outputs its labels, so that columns do not need to be in order. 

The idea was to create the appropriate ROS style messages for each thing being publish, as in, say a force, we publish a vector, is it a jointstate, then jointstate messages, and so on. This has proven difficult and was relegated to a later stage of integration. 

As such, this design pattern, i.e. the common pipe, was created to ease implementation. 

To read more inputs, message filters were created and to allow this more easily, the basic pipe also uses message filters, but with a single topic being read. 

A further improvement would be to create sinks and sources which could be easily added, but this is perhaps not wanted/necessary, since we hope to create the appropriate ROS style topics in the future.

## Design pattern: the opensimrt\_bridge package

To allow conversion between these topic types an additional package was created, the opensimrt\_bridge package, which both reads files and publishes them as sources and converts (planned implementation, not currently finished) CommonStamped to ROS style topics which can be integrated into ROS and visualized in Rviz. 

We expect this to increase latency and it is undesirable, but we think this is necessary during debug phase.

