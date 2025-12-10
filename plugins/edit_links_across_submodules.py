"""
Replaces the edit_url for documentation included in other submodules.

For example, the file robot_parameters.md [1] is included using the submodule located at repos/stretch_body

When edit_uri is defined in the mkdocs.yml, the edit link for that page is
https://github.com/hello-robot/hello-robot.github.io/blob/0.3/repos/stretch_body/docs/robot_parameters.md
which does not link properly.

The code contained in this module is inspired by the submodule-edit-uri plugin [2]
and uses the mkdocs hooks [3] functionality.

It reads the submodules from `.gitmodules` using GitPython during the configuration step,
and then substitutes the proper url in the full output.

The resulting link is now correct, linking to the full github blob url [1]

[1] https://github.com/hello-robot/stretch_body/blob/master/docs/robot_parameters.md
[2] https://github.com/sondregronas/mkdocs-submodule-edit-uri
[3] https://www.mkdocs.org/user-guide/configuration/#hooks


"""
from git import Repo


def urljoin(*a):
    s = ''
    for sub in a:
        if s and not s.endswith('/'):
            s += '/'
        s += sub
    return s


def on_config(config):
    submod_d = {}
    config['submodules'] = submod_d
    repo = Repo('.')
    for submod in repo.submodules:
        url = submod.url
        if url.endswith('.git'):
            url = url[:-4]
        submod_d[submod.name] = {
            'path': submod.path,
            'url': url,
            'branch': submod.branch.name,
        }


def on_post_page(output, page, config):
    # Assume that the edit uri is (blob|edit)/(branch_name)/docs/
    edit_pieces = config['edit_uri'].split('/')
    assert len(edit_pieces) == 4, config['edit_uri']
    verb = edit_pieces[0]
    branch = edit_pieces[1]

    base = urljoin(config['repo_url'], verb, branch)

    for name, submod in config['submodules'].items():
        old_path = urljoin(base, submod['path'])
        new_path = urljoin(submod['url'], verb, submod['branch'])
        output = output.replace(old_path, new_path)
    return output
