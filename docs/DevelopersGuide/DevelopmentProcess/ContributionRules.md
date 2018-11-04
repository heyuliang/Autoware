# Contribution Rules

**You must follow the rules described below as part of the Autoware community.**
Please read the following carefully before you start contributing to Autoware.
Thank you for your time.

## To Begin With

Please join Autoware Slack at [https://autoware-developer.slack.com/](https://autoware-developer.slack.com/) and say hello to the community. You can also post your message on Autoware Googlegroups at [autoware@googlegroups.com](mailto:autoware@googlegroups.com). If you want to subscribe to it, please click the "Apply to Join Group" button at [https://groups.google.com/d/forum/autoware](https://groups.google.com/d/forum/autoware). For those who do not have Google accounts, please send an email directly to [autoware+subscribe@googlegroups.com](mailto:autoware+subscribe@googlegroups.com). Finally, if you do not have a GitHub account, you can create one at [https://github.com/join](https://github.com/join).

## Development Workflow

To assure traceability in your code, please follow our development process:
* If you are working on a feature/modify-node, create an issue first.
* Create a branch, work on the feature/modify-node, refer in at least one commit to the issue #number.
* Open a pull request for the given feature/modify-node and fill out the pull request template. 

This will assure that you know where the feature originated from and the implemented code will be linked to the feature request (or bug report). Otherwise there is absolutely no traceability.
Secondly, with the pull request template it will be easier for the reviewer(s) to understand the changes and we can later on convert "Steps to Reproduce" into integration tests.

## Branching Model

In order to make the development process efficient, we ask you to comply with the branching model described below.
On this model, we mainly use six branches - master, develop, feature, release, experimental, and hotfix.

### master

This is the latest stable version of Autoware.

### develop and feature

In general, developers should NOT work on the "develop" branch.
When you develop new functions, please check out a new branch, "feature/[branch_name]", from the "develop" branch, and modify the code there.
After the modificaiton, please send a pull request to the "develop" branch.

### release

This situation is true of only the persons in charge of releasing Autoware.
Once we complete some major development, we make a new release.
For the release work, please checkout a new branch, "release/[1.xx.yy], from the "develop" branch, and modify the code there.
After the modification, please send a pull request: "xx" in version of release/[1.xx.yy] is increased when checked out from the "develop" branch, and yy is also increased when bug fixes are done.
Finally, we merge this branch to the master branch, attaching a version tag.

### experimental

If your contribution is not a new feature but is to change one of the existing branches, please checkout a new branch, "experimental/[branch_name]", from the corrensponding branch. Please discuss with other contributions if this change can be merged to the original branch.

### hotfix

If we encounter bugs in the "master" branch after the release, we check out the "hotfix" branch from the "master" branch and fix the bugs there.
This branch is merged to each corresponding branch - master, release, and develop.
After the merge, the version of the master and the release branches is increased.

Reference for the git-flow model
- http://nvie.com/posts/a-successful-git-branching-model/

## Pull Request

When you are ready to send a pull request from your branch, please follow:

1. A design article should be posted with a GitHub comment for every feature or bug. 
1. Every feature/bug implementation needs to be thoroughly reviewed (at least two reviewers). You can specify your favorite or appropriate reviewers by @accountname.
1. A sample program for the unit test needs to be submitted so that the reviewers or others can check if the implementation logic is correct.
1. The integration test with the [demo data](https://github.com/CPFL/Autoware/wiki/Demo-Data) needs to be passed.  
1. Coding style enforcement must be applied: e.g., [cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint).
1. The reviewers would further run static code analysis: e.g., [cppcheck](http://cppcheck.sourceforge.net/).

We introduce [Travis CI](https://travis-ci.org/) to automate the above test and deploy steps.

## Coding Standards

The following are regarding coding standards preferred in Autoware.
We know that you must have your own coding style, but please respect our standards in our community.
No need to throw away your coding style, just do your best to follow our standards.

### ROS C++ Coding

First of all, please understand the ROS coding standards before you add any new code to Autoware.

* [ROS Developers Guide](http://wiki.ros.org/DevelopersGuide)

* [ROS C++ Coding Style](http://wiki.ros.org/CppStyleGuide)

You might be interested in using [ROS clang-format](https://github.com/davetcoleman/roscpp_code_format) that helps you to comply with the ROS C++ coding standards automatically in terms of styles, such as indent size and brackets space.

#### How to use clang-format

* Install clang-format. A newer version is better. Ubuntu has a package:
```
sudo apt-get install clang-format-x.x
```
* Locate the .clang-format file at the top directory.
* Apply clang-format to the target source file:
```
clang-format -i filename
```

Be careful that `clang-format -i` will overwrite the file. It is safe to do "git commit" before playing with clang-format.
If you want to apply clang-format to the entire system, run the following script:
```sh
for file in $(git ls-files | \grep -E '\.(c|cpp|h|hpp)$' | \grep -v -- '#')
do
    clang-format -i $file
done
```

### ROS Python Coding

In addition to C++, you will often use Python in ROS. There is also a coding style guide for Python recommended in ROS.

* [ROS Python Coding Style](http://wiki.ros.org/PyStyleGuide)

You can use [pep8](https://pypi.python.org/pypi/pep8) as a tool to check PEP8-compliant coding.
Therefore many existing ROS programs that use Python 2.5, but Ubuntu 16.04 or later versions will use Python 3 by default.
Considering maintenance of coding in the future, thus, Python 3 is preferred in Autoware.

### Notes for Package and Library Development

* Algorithms should be implemented in libraries as much as possible. For example, the normal distributions transform (NDT) algorithm could be provided as something like libndt_xxx. Library distribution allows this algorithm module to be used for other applications than ROS or Autoware. This is a spirit of open source.

* Do not make unnecessary dependencies among libraries. In particular, never make circular dependencies. This jeopardizes the entire build system.

* Do not include header files generated from msg files of other packages.

* Keep every library independent and general. Creating too many libraries is also a bad idea.

* Provide a sample program to test the functions of library. 

### Notes for Design and Implementation

#### Global Variables

You should not use global variables unless they are really needed. Instead, you should use classes or structs to hold variables. Even for libraries, you do not recommend using global variables. In C++, you can use methods. In C, you can use pointers or references for function arguments.

Besides in using global variables, you should take care of thread-safe implementation for multi-threaded programs as global variables may be accessed simultaneously among threads. In ROS, particularly, there are many other threads running in background (e.g., polling threads for subscribing to topics). Thus, you should avoid using global variable as much as possible, though you can use mutual exclusion to ensure thread-safe implementation if you really need global variables.

#### Arguments and Return Values

Function calls without arguments or without return values (i.e., void types) are difficult to test, because the results of function calls are all indirect and not visible from the function callees. Therefore you should make functions declared with specific arguments and meaningful return values so that a unique set of arguments always leads to the same result.

#### Naming

Function names must represent what these functions do. For example, `init()` or `destroy()` is not an appropriate name, because they do not tell what they initialize or destroy. Such a short and simple function name may also likely cause symbol name conflicts among multiple libraries. Function naming should be discussed when new libraries are added to Autoware. The following are some tips to solve this function naming problem.

1. Use a library name as prefix. For example, if the `fusion` library wants to export `init()` or `destroy()`, they should be named as `fusion_init()` or `fusion_destroy()`.

1. Use namespace. You can wrap the entire code of the `fusion` library by `namespace autoware::fusion {}`. This way, you can identify these functions by `autoware::fusion::init()` or `autoware::fusion_init()`. In fact, Autoware is desired to identify all the libraries, packages, and topics by namespace so that partial pieces of Autoware can be used safely in other projects.

### Export Symbols

You should clarify what symbols are exported, and should not export those that would not be used or referenced by other packages and libraries. If you want not to export symbols, please use unnamed namespace or private members in classes in C++. In C, whereas, please use `static` that protects the corresponding symbols in local files.

## Notes for Timing Constraints 

* Do not publish topics in random periods.

* Basically, topics must be published once updated. That is to say, you should publish topics in callback functions.
The following is a bad example of coding.
```
while(ros::ok()) {
    publisher.publish(xxx);
    loop.sleep();
}
```
* If a node has two or more topics, it has to publish them timely when all of them are ready. For example, if you subscribe to A and B topics, do not publish in the callback function associated with A, where only A is updated. You should wait for both A and B to be updated. The following is sample code:   
```
A_callback(A_msg) {
    if (is_b_callback == true) { // If A was updated 
        publish(xxx); // publish the topic
        is_a_callback = false;
        is_b_callback = false;
        return;
    }
    is_a_callback = true;
}
B_callback(B_msg) {
    if (is_a_callback == true){
        publish(xxx);
        is_a_callback = false;
        is_b_callback = false;
        return;
    }
    is_b_callback = true;
}
```

* Always put a header in the topic, and inherit the time stamp from the preceding topic. Do not update the header's time stamp without inheritance. If a node has two or more topics, you can inherit the time stamp from any of these, because their time stamps are supposed to be synchronized.
 
* Do not use both "service" and "topic" at the same time. If they co-exist, timing estimates become more difficult. In most cases, you should use "topic" rather than "service". However, you may use "service" for utility and interface packages, which do not require real-time performance unlike perception, planning, and control packages.

* Do not use "topic" for the large size of data, but use "nodelet" in this case. Large topic data, such as images and pointcloud scans, would sacrifice a few milliseconds to serialize and deserialize.

* Do not use "MultiThreadSpin". It is not preferable from the point of view of real-time scheduling, because timing estimates and resource allocation become more difficult. 

* Do not use `output="screen"`. It is okay for the debugging purpose, however, please remove `output="screen"` before you commit to the "develop" branch - you never want to annoy your colleagues by flooding terminal information. To monitor information, basically, we prefer ROS_INFO and ROS_DEBUG to rqt, but rqt is definitely useful for the debugging purpose. So you can use it, but just be noted that you should remove it before you commit to the "develop" branch.

* Avoid using "tf" as much as possible. For example, you can obtain the local position from the "current_pose" topic, and do not really need to use "tf". In fact, the "tf" library and ROS are disjoint (very often used together, though). Using "tf" makes timing estimates more difficult. Instead of "tf", use "topic" as much as possible. Frankly speaking, "tf" is useful for applications such as arm robots with many joints, which require dynamic transformation of coordinates, but is not very useful for self-driving vehicles because transformation of coordinates can be often statically determined. 

## Notes for Embedded Platforms

* Do not use a wide variety of libraries. It will decrease portability of RTOS. For example, use ros::WallTime rather than the chrono library. However, what about the boost library? It remains as an open question...

* For function arguments, use pointers and const calls by reference as much as possible. It is not necessary to use them for int or double arguments, but for vector or array arguments, you should use const calls by reference. It saves memory footprint, and also reduces overhead of the function call.

* Use the reference argument when you return from the function. A direct return value will degrade performance. However, be careful about the scope of pointer and so on. Basically, you may want to use a direct return value just for error numbers or Boolean results.

* Avoid dynamic partitioning, such as malloc and new. malloc and new could cause memory leaks. In addition, they make unclear the amount of used resources.

* If the size of vector is roughly estimated, use reserve. The vector allocates memory regions twice in case of capacity shortage. It will require a large amount of time to allocate memory regions twice, you had better to use reserve so that the required memory regions can be allocated tightly in advance.

* Avoid a monster function that spans more than 50 lines. Basically, any function should be kept around 20-30 lines of code. In addition, bear in mind that the granularity of coding within the function should be well balanced. According to [Effective C++](https://www.amazon.com/Effective-Modern-Specific-Ways-Improve/dp/1491903996/ref=sr_1_2?ie=UTF8&qid=1515426637&sr=8-2&keywords=effective+C%2B%2B), a method or function of sophisticated code has only 14 lines on average.
 
### Bad Example
```
callback() {
    start_time = clock_get_time(); // Not abstracted at all 
    compute_xxx(); // Abstracted too much
}
```
* Use inline effectively. Such a function that has a single line of code, for instance, should be inline. However, inline functions will enlarge footprint. So be careful about using too many inline functions.

* Function naming should correspond to the function code. For example, do not write heavy code in get() or set(), because these functions are supposed to just get or set some values. Function naming should imply what the function is and what is the cost of processing time. If you want to create a time-consuming function, for example, probably function naming such as compute_xxx is suitable. 

## C++ Books

C++ 11/14/17 has introduced many useful capabilities, e.g., type inference.
You may want to review C++ 11 through the popular books: [[Amazon links](https://www.amazon.com/s/ref=nb_sb_noss_2/135-5470609-2801335?url=search-alias%3Daps&field-keywords=C%2B%2B+11)]
