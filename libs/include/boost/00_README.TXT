Boost is mostly a header library thus there is no need to
worry about linking to the lib binaries. Just
include the header you need to your project.

The only Boost libraries that must be built separately are:

    * Boost.Filesystem
    * Boost.GraphParallel
    * Boost.IOStreams
    * Boost.MPI
    * Boost.ProgramOptions
    * Boost.Python (see the Boost.Python build documentation before building and installing it)
    * Boost.Regex
    * Boost.Serialization
    * Boost.Signals
    * Boost.System
    * Boost.Thread
    * Boost.Wave

A few libraries have optional separately-compiled binaries:

    * Boost.DateTime has a binary component that is only needed if you're using its to_string/from_string or serialization features, or if you're targeting Visual C++ 6.x or Borland.
    * Boost.Graph also has a binary component that is only needed if you intend to parse GraphViz files.
    * Boost.Math has binary components for the TR1 and C99 cmath functions.
    * Boost.Random has a binary component which is only needed if you're using random_device.
    * Boost.Test can be used in “header-only” or “separately compiled” mode, although separate compilation is recommended for serious use.


In order to keep the project lightweight only 
few modules of the boost 1.46.1 framwork is here.
Feel free to add other modules.