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
## Editing content
In order to make a small change that doesn't require a version bump (for example):
```commandline
cd ~/repos/hello-robot.github.io
git checkout 0.1

<make edits>

git add *
git commit -m 'make some edits'
git push
```

## Previewing Edits

```commandline
cd ~/repos/hello-robot.github.io
mike serve
Starting server at http://localhost:8000/
Press Ctrl+C to quit.
```
You can then preview the content at site http://localhost:8000
## Deploying Edits
If, for example, the version under edit is 0.1, you can push the edits to the webserver by:
```commandline
cd ~/repos/hello-robot.github.io
mike deploy 0.1 --push
```
This will push the changes to the `gh-pages` branch and the edits will be reflected at [docs.hello-robot.com](docs.hello-robot.com). 

**Note**: It may take time / browser refreshes for the changes to be correctly reflected at [docs.hello-robot.com](docs.hello-robot.com),

## Making a new version
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

Note the format for the title (-t). Keep a consistent naming structure. Also note that this version is aliased as `experimental`. 
## Other Tips
The currently hosted versions that are managed by mike can be found with
```commandline
~/repos/hello-robot.github.io$ mike list
"0.2: Beta (WIP)" (0.2) [beta]
"0.1: Latest" (0.1) [latest]
```
The default version that is present at is the one with the alias `latest` as defined in mkdocs.yaml:
```commandline
extra:
  version:
    provider: mike
    default: latest
```
To release a version update its alias to `latest`. Also update its title to accurately reflect this. See the [Mike documentation](https://pypi.org/project/mike/) for more information.

