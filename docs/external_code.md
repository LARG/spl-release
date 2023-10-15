#Guidelines for including external code

Often, we will make use of external code or packages. There are many ways of doing so, so follow these best practices for maintainable code.

##Permissions

First, make sure that there are no issues with licensing. This is usually not an issue for robocup, since our code is used for research purposes. If however the code is from another team's repository, we must declare this before the competition. Make sure proper attribution is given.

##Including Built Libraries

When we are using an pre-built external library, the preferred method is to include the dependency in the `package.xml` for the package that requires it. This system has two advantages:
1. [rosdep](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Rosdep.html) can automatically handle installing these dependencies. Our install script calls rosdep automatically!
2. Doing it this way means that the dependencies live close to the package that needs them. If later on, that package is no longer needed, the dependencies will be automatically removed as well!

If this isn't possible (e.g., files not hosted on repos) then the second choice is to include the installation as part of the install script.

Lastly, if neither of those options are possible (e.g., for dependencies required by the install script itself), add it to the documentation as part of the installation procedure with clear instructions on how to install the necessary dependency.

##Including Source Code

Sometimes it is necessary to include code that is built from scratch. Usually, it's for one of the following reasons:
* We have to build this dependency from source with a particular set of flags.
* We have a fork of this repo that we have modified in some way.

Note: If it's not one of the above reasons, it's probably better to include it as a built library.

###Git Submodule

If the exteral repo is a git repo, we can include it as a git submodule. This is usually the best option, because it makes updating the dependency very easy.
```
git submodule add <git clone url>
```

There is lots of info online about working with submodules.

###The Install Script

If it is not possible to make it a git submodule, consider adding it to the install script. This is a better option than including a copy of the code within our repo because:
* It reduces clutter in our repo.
* It preserves the link to the original code, so the code can be updated in the future.

The install script should download the code. The install targets are intended to only be built once. Other targets in the build script are checked every time the build script is run in case something has changed.

An even more extreme option is to build the external code once, host it somewhere online, and then have rosdep or the install script pull down this custom-built code. This can save time for large libraries that take a long time to compile; however, this option should be used sparingly. Chances are that a library that needed to be custom-built will need to be custom-built again (perhaps for a different architecture or operating system). Making that part of the install or build script makes that easier to do in the future.

###Include it in our repo directly (last resort)

Sometimes, the most sensible option is just to include a copy of some code within our own repo. This is usually the case for small bits of code that require heavy modification. In this case, it is very important to leave comments clearly indicating which parts of the code were taken from an external repository and where that external repository is located.
