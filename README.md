# Stretch Documentation Web Server

This repository generates the documentation hosted at [docs.hello-robot.com](https://docs.hello-robot.com).

## Setup for Documentation Development
In order to create a development environment on a 20.04 Ubuntu machine:
```
cd ~/repos
git clone https://github.com/hello-robot/hello-robot.github.io
cd hello-robot.github.io
./install.sh
```
## Branch Structure
The documentation is organized on versioned branches:
```commandline
/repos/hello-robot.github.io$ git branch
  0.1
  0.2
  gh-pages
```
This allows for the development of new documentation under a new version while hosting the stable documentation as default.

All content maintained under the versioned branch. This master branch is for hosting install scripts and this README.md only.

The branch `gh-pages` is a special orphan branch that the documentation build is pushed to using the Mike versioning plug-in for MkDocs. This branch is hosted by default at via GitHub Pages at [docs.hello-robot.com](docs.hello-robot.com),

## Listing Versions
The currently hosted versions that are managed by mike can be found with
```commandline
~/repos/hello-robot.github.io$ mike list
"0.2: Beta (WIP)" (0.2) [beta]
"0.1: Latest" (0.1) [latest]
```
## Editing content
In order to make a small change that doesn't require a version bump (for example):
```commandline
cd ~/repos/hello-robot.github.io
git checkout 0.1

<make edits>

mike deploy 0.1 --push

git add *
git commit -m 'make some edits'
git push
```

This will push the changes to the `gh-pages` branch and the edits will be reflected at [docs.hello-robot.com](docs.hello-robot.com). 

**Note**: It may take time / browser refreshes for the changes to be correctly reflected at [docs.hello-robot.com](docs.hello-robot.com),

## Other Tips
To make a new version, for example:
```commandline
cd ~/repos/hello-robot.github.io
git pull
git checkout 0.2
git branch 0.3
git checkout 0.3

<make edits>

mike deploy 0.3 'experimental' -t '0.3: Experimental development' --push
```


```
./deploy.sh
```
