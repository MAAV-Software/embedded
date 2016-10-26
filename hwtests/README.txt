To add a new Hardware Test Case

1. In hwtests/tests, create a folder for your test case. Name it something
   short, recognizable, and meaningful. 

2. Copy the contents of hwtests/tests/led into your new folder

3. Edit hwtests/tests/yourtest/CMakeLists.txt with the name of the test on 
   line 13

4. Edit CoverageManifest.txt with the files from the main codebase the test 
   case depends on (i.e., what it is test and its dependencies)

5. Edit hwtests/tests/CMakeLists.txt to add your test case as a subdir
