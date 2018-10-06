# AStarEngine

```
    _     ___ __                 _____
   / \   / _// /_ __   ___  __  / ___/__  ___
  / _ \ _\ \/ __/ _ \_/ _/ /_/ / __// _ \/ _ \
 /_/ \_\\__/__/ \____/_/       \___/_//_/\_  /
                                         \__/
```
AStarEngine is an implementation of [A-Start](http://theory.stanford.edu/~amitp/GameProgramming/) algorithm in C++.
This project is built with reference to Ref.[2], thus is similar to Ref.[2] in the main structure.

## Usage

First, you need to inherit the *AANode* class and the *AAGrid* class according
to your problem requirements, and then generate a *AAEngine* instance class with
the inherited classes. To execute the program, you just need to set the start
and goal nodes, and call the `Search()` methods. Noted that there are 3 ways to
set the start and goal nodes, 1) by the constructor of *AAEngine* instance class,
2) through the `SetStartAndGoalNodes(const NodeType&, const NodeType&)` 
method, and 3) through the `Search(const NodeType&, const NodeType&)` method.

See the test code for more details.

## References

[1] https://github.com/justinhj/astar-algorithm-cpp
[2] https://github.com/vandersonmr/A_Star_Algorithm
[3] http://code.google.com/p/recastnavigation/ [^1]

[^1] For the convenience of users who cannot access Google, I've forked Ref.[3] from Google.
     Check it at my [repository](https://github.com/weiquanmao/policy-based-search).

---

By [WeiQM](https://weiquanmao.github.io/).