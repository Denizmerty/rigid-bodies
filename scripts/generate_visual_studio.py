"""Regenerate the Visual Studio test and tool projects and the solution file.

The product's projects (visualstudio/*.vcxproj and visualstudio/thirdparty/*.vcxproj) are
maintained by hand, in Visual Studio or a text editor, and the CMake build reads their file lists.
The tests and command-line tools are defined in CMakeLists.txt; this script turns CMake's model of
them into native projects under visualstudio/tests and visualstudio/tools, then rewrites
RigidBodies.sln around every project. Run it after adding, removing or changing a test or tool:

    .\\build.ps1 -Tests                     # configures CMake with tests and tools
    python scripts/generate_visual_studio.py

`--check` reports whether the committed projects and solution are current without writing them.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import uuid

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
VS_DIR = os.path.join(ROOT, 'visualstudio')
SOLUTION = os.path.join(ROOT, 'RigidBodies.sln')
CPP_PROJECT_TYPE = '{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}'
FOLDER_TYPE = '{2150E333-8FDC-42A3-9474-1A3956D46DE8}'
CONFIG = "'$(Configuration)|$(Platform)'=='Release|x64'"

# Product projects, in the order Solution Explorer lists them, with their solution folder. The
# application comes first: Visual Studio starts the first project of a solution it opens afresh.
PRODUCT_PROJECTS = [
    ('RigidBodies.App.vcxproj', 'Application'),
    ('RigidBodies.AppCore.vcxproj', 'Application'),
    ('RigidBodies.Ui.vcxproj', 'Application'),
    ('RigidBodies.Render.vcxproj', 'Application'),
    ('RigidBodies.Core.vcxproj', 'Application'),
    ('RigidBodies.Physics.vcxproj', 'Simulation'),
    ('RigidBodies.Math.vcxproj', 'Simulation'),
    ('thirdparty/FreeType.vcxproj', 'Third party'),
    ('thirdparty/RmlUi.vcxproj', 'Third party'),
]

# CMake target -> product project, for references and link libraries.
TARGET_PROJECTS = {
    'RigidBodies.Math': ('RigidBodies.Math.vcxproj', 'RigidBodies.Math.lib'),
    'RigidBodies.Physics': ('RigidBodies.Physics.vcxproj', 'RigidBodies.Physics.lib'),
    'RigidBodies.Core': ('RigidBodies.Core.vcxproj', 'RigidBodies.Core.lib'),
    'RigidBodies.Render': ('RigidBodies.Render.vcxproj', 'RigidBodies.Render.lib'),
    'RigidBodies.Ui': ('RigidBodies.Ui.vcxproj', 'RigidBodies.Ui.lib'),
    'RigidBodies.AppCore': ('RigidBodies.AppCore.vcxproj', 'RigidBodies.AppCore.lib'),
    'freetype': ('thirdparty/FreeType.vcxproj', 'freetype.lib'),
    'rmlui': ('thirdparty/RmlUi.vcxproj', 'rmlui.lib'),
}

SYSTEM_LIBRARIES = ['kernel32.lib', 'user32.lib', 'gdi32.lib', 'winspool.lib', 'shell32.lib', 'ole32.lib',
                    'oleaut32.lib', 'uuid.lib', 'comdlg32.lib', 'advapi32.lib']

# Definitions the shared property sheets already supply.
SHARED_DEFINITIONS = {'_CRT_SECURE_NO_WARNINGS', 'WIN32', '_WINDOWS', 'NDEBUG'}

# Executables CMake defines that the native solution does not build, with the reason.
SKIPPED_TARGETS = {
    # The developer overlay exists only in CMake's Debug and RelWithDebInfo configurations.
    'RigidBodies.DeveloperOverlay.Tests',
}

SOLUTION_ITEM_FOLDERS = [
    ('Build', ['CMakeLists.txt', 'CMakePresets.json', 'build.ps1', 'clean.ps1', 'cmake/*.cmake',
               'visualstudio/*.props', 'visualstudio/*.targets', '.clang-format', '.clang-tidy', '.editorconfig',
               '.gitattributes', '.gitignore', '.github/workflows/*.yml', '.github/CODEOWNERS',
               '.github/rulesets/*.json']),
    ('Documentation', ['README.md', 'LICENSE', 'docs/*.md', 'assets/README.md', 'assets/scenarios/README.md', 'vendor/README.md']),
    ('Scripts', ['scripts/*.ps1', 'scripts/*.py', 'packaging/*']),
    ('Test support', ['tests/support/*.hpp']),
]


def project_guid(key):
    return '{%s}' % str(uuid.uuid5(uuid.NAMESPACE_URL, 'https://github.com/Denizmerty/rigid-bodies/' + key)).upper()


def windows_path(path):
    return path.replace('/', '\\')


def under_root(path):
    """$(RigidBodiesRoot)-relative form of an absolute path inside the repository, else the path."""
    normal = os.path.normcase(os.path.normpath(path))
    root = os.path.normcase(ROOT)
    if normal == root:
        return '$(RigidBodiesRoot)'
    if normal.startswith(root + os.sep):
        return '$(RigidBodiesRoot)' + windows_path(os.path.relpath(os.path.normpath(path), ROOT))
    return windows_path(path)


# ----------------------------------------------------------------------------------------------
# CMake's model of the tests and tools
# ----------------------------------------------------------------------------------------------

def find_cmake():
    cmake = shutil.which('cmake')
    if cmake:
        return cmake
    bundled = os.path.join(os.environ.get('ProgramFiles', r'C:\Program Files'), 'Microsoft Visual Studio')
    if os.path.isdir(bundled):
        for version in sorted(os.listdir(bundled), reverse=True):
            for edition in ('Enterprise', 'Professional', 'Community', 'BuildTools', 'Preview'):
                candidate = os.path.join(bundled, version, edition, r'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe')
                if os.path.isfile(candidate):
                    return candidate
    sys.exit('cmake was not found on PATH or in a Visual Studio installation')


def load_codemodel(build_dir):
    query = os.path.join(build_dir, '.cmake', 'api', 'v1', 'query', 'codemodel-v2')
    if not os.path.isfile(os.path.join(build_dir, 'CMakeCache.txt')):
        sys.exit(f'{build_dir} is not a configured CMake build directory; run .\\build.ps1 -Tests first')
    cache = open(os.path.join(build_dir, 'CMakeCache.txt'), encoding='utf-8').read()
    for option in ('RIGIDBODIES_BUILD_TESTS', 'RIGIDBODIES_BUILD_TOOLS', 'RIGIDBODIES_BUILD_APP'):
        if not re.search(r'^%s:BOOL=ON$' % option, cache, re.M):
            sys.exit(f'{build_dir} must be configured with {option}=ON (.\\build.ps1 -Tests uses such a configuration)')
    os.makedirs(os.path.dirname(query), exist_ok=True)
    open(query, 'a').close()
    subprocess.run([find_cmake(), '-S', ROOT, '-B', build_dir], check=True, stdout=subprocess.DEVNULL)
    reply = os.path.join(build_dir, '.cmake', 'api', 'v1', 'reply')
    index = sorted(name for name in os.listdir(reply) if name.startswith('index-'))[-1]
    index = json.load(open(os.path.join(reply, index), encoding='utf-8'))
    codemodel = index['reply']['codemodel-v2']['jsonFile']
    codemodel = json.load(open(os.path.join(reply, codemodel), encoding='utf-8'))
    configurations = {c['name']: c for c in codemodel['configurations']}
    configuration = configurations.get('Release') or codemodel['configurations'][0]
    targets = {}
    for entry in configuration['targets']:
        targets[entry['id']] = json.load(open(os.path.join(reply, entry['jsonFile']), encoding='utf-8'))
    return targets, codemodel['paths']


def describe_executable(target, targets, paths):
    source_dir = paths['source']
    build_dir = paths['build']
    group = target['compileGroups'][0]
    sources = []
    for source in target['sources']:
        if 'compileGroupIndex' not in source:
            continue
        path = source['path']
        if not os.path.isabs(path):
            path = os.path.join(source_dir, path)
        sources.append(os.path.normpath(path))
    include_dirs = []
    for include in group.get('includes', []):
        path = os.path.normpath(include['path'])
        if os.path.normcase(path).startswith(os.path.normcase(os.path.normpath(build_dir))):
            raise SystemExit(f"{target['name']} includes the generated directory {path}, which the native build lacks")
        include_dirs.append(under_root(path))
    definitions = []
    root_forward = ROOT.replace('\\', '/')
    for define in group.get('defines', []):
        text = define['define']
        if text.split('=', 1)[0] in SHARED_DEFINITIONS:
            continue
        definitions.append(text.replace(root_forward, '$(RigidBodiesRootForward)'))
    libraries = []
    for fragment in target.get('link', {}).get('commandFragments', []):
        if fragment.get('role') != 'libraries':
            continue
        for item in fragment['fragment'].split():
            item = item.strip('"')
            name = os.path.basename(item)
            if name in SYSTEM_LIBRARIES:
                if '$(RigidBodiesSystemLibraries)' not in libraries:
                    libraries.append('$(RigidBodiesSystemLibraries)')
                continue
            known = [lib for _, lib in TARGET_PROJECTS.values()]
            if name in known:
                libraries.append('$(RigidBodiesLibraryDir)' + name)
            elif name.lower() == 'sdl3.lib':
                libraries.append('$(RigidBodiesSdlLibrary)')
            elif item.startswith('/') or item.startswith('-'):
                continue
            else:
                raise SystemExit(f"{target['name']} links {item}, which the generator does not know")
    # CMake lists dependencies in no stable order, so the references follow TARGET_PROJECTS instead.
    dependencies = {targets[dependency['id']]['name'] for dependency in target.get('dependencies', [])}
    references = [name for name in TARGET_PROJECTS if name in dependencies]
    output = target.get('nameOnDisk') or target['name'] + '.exe'
    return {
        'name': target['name'],
        'output': os.path.splitext(output)[0],
        'sources': sources,
        'include_dirs': include_dirs,
        'definitions': definitions,
        'libraries': libraries,
        'references': references,
        'uses_sdl': '$(RigidBodiesSdlLibrary)' in libraries,
    }


def project_location(description):
    name = description['name']
    if name.endswith('.Tests'):
        first = os.path.relpath(description['sources'][0], ROOT).replace('\\', '/')
        match = re.match(r'tests/RigidBodies\.([A-Za-z]+)\.Tests/', first)
        group = match.group(1) if match else 'Other'
        return os.path.join(VS_DIR, 'tests', group, name + '.vcxproj'), 'Tests\\' + group
    return os.path.join(VS_DIR, 'tools', name + '.vcxproj'), 'Tools'


def executable_project(description, project_path, kind):
    project_dir = os.path.dirname(project_path)
    lines = []
    add = lines.append
    add('<?xml version="1.0" encoding="utf-8"?>')
    add('<!-- Generated by scripts/generate_visual_studio.py from the CMake target %s. -->' % description['name'])
    add('<Project DefaultTargets="Build" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">')
    add('  <ItemGroup Label="ProjectConfigurations">')
    add('    <ProjectConfiguration Include="Release|x64">')
    add('      <Configuration>Release</Configuration>')
    add('      <Platform>x64</Platform>')
    add('    </ProjectConfiguration>')
    add('  </ItemGroup>')
    add('  <PropertyGroup Label="Globals">')
    add('    <VCProjectVersion>18.0</VCProjectVersion>')
    add('    <Keyword>Win32Proj</Keyword>')
    add('    <ProjectGuid>%s</ProjectGuid>' % project_guid(description['name']))
    add('    <ProjectName>%s</ProjectName>' % description['name'])
    add('    <RootNamespace>%s</RootNamespace>' % description['name'].replace('.', ''))
    add('    <WindowsTargetPlatformVersion>10.0</WindowsTargetPlatformVersion>')
    add('  </PropertyGroup>')
    add('  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.Default.props" />')
    add('  <PropertyGroup Condition="%s" Label="Configuration">' % CONFIG)
    add('    <ConfigurationType>Application</ConfigurationType>')
    add('    <UseDebugLibraries>false</UseDebugLibraries>')
    add('    <PlatformToolset>$(DefaultPlatformToolset)</PlatformToolset>')
    add('    <WholeProgramOptimization>false</WholeProgramOptimization>')
    add('    <CharacterSet>NotSet</CharacterSet>')
    add('  </PropertyGroup>')
    add('  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.props" />')
    add('  <ImportGroup Label="PropertySheets" Condition="%s">' % CONFIG)
    props = os.path.relpath(VS_DIR, project_dir)
    add('    <Import Project="%s" />' % windows_path(os.path.join(props, 'RigidBodies.props')))
    add('    <Import Project="%s" />' % windows_path(os.path.join(props, 'RigidBodies.Code.props')))
    add('  </ImportGroup>')
    add('  <PropertyGroup Label="UserMacros" />')
    add('  <PropertyGroup Condition="%s">' % CONFIG)
    add('    <OutDir>$(RigidBodiesOutputDir)%s\\</OutDir>' % kind)
    add('    <TargetName>%s</TargetName>' % description['output'])
    if description['uses_sdl']:
        add('    <RigidBodiesStageSdlRuntime>true</RigidBodiesStageSdlRuntime>')
    add('    <LocalDebuggerWorkingDirectory>$(OutDir)</LocalDebuggerWorkingDirectory>')
    add('    <DebuggerFlavor>WindowsLocalDebugger</DebuggerFlavor>')
    add('  </PropertyGroup>')
    add('  <ItemDefinitionGroup Condition="%s">' % CONFIG)
    add('    <ClCompile>')
    if description['include_dirs']:
        add('      <AdditionalIncludeDirectories>%s;%%(AdditionalIncludeDirectories)</AdditionalIncludeDirectories>' % ';'.join(description['include_dirs']))
    if description['definitions']:
        add('      <PreprocessorDefinitions>%s;%%(PreprocessorDefinitions)</PreprocessorDefinitions>' % ';'.join(description['definitions']))
    add('    </ClCompile>')
    add('    <Link>')
    add('      <SubSystem>Console</SubSystem>')
    add('      <AdditionalDependencies>%s</AdditionalDependencies>' % ';'.join(description['libraries']))
    add('    </Link>')
    add('  </ItemDefinitionGroup>')
    add('  <ItemGroup>')
    for source in description['sources']:
        add('    <ClCompile Include="%s" />' % windows_path(os.path.relpath(source, project_dir)))
    add('  </ItemGroup>')
    if description['references']:
        add('  <ItemGroup>')
        for reference in description['references']:
            relative, _ = TARGET_PROJECTS[reference]
            path = os.path.relpath(os.path.join(VS_DIR, windows_path(relative)), project_dir)
            name = os.path.splitext(os.path.basename(relative))[0]
            add('    <ProjectReference Include="%s">' % windows_path(path))
            add('      <Project>%s</Project>' % project_guid(name))
            add('      <LinkLibraryDependencies>false</LinkLibraryDependencies>')
            add('    </ProjectReference>')
        add('  </ItemGroup>')
    add('  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.targets" />')
    add('  <Import Project="%s" />' % windows_path(os.path.join(props, 'RigidBodies.targets')))
    add('</Project>')
    return '\r\n'.join(lines) + '\r\n'


# ----------------------------------------------------------------------------------------------
# Solution
# ----------------------------------------------------------------------------------------------

def read_guid(project_path):
    text = open(project_path, encoding='utf-8-sig').read()
    match = re.search(r'<ProjectGuid>(\{[0-9A-Fa-f-]+\})</ProjectGuid>', text)
    if not match:
        raise SystemExit(f'{project_path} has no ProjectGuid')
    return match.group(1).upper()


def solution_items(patterns):
    import glob
    files = []
    for pattern in patterns:
        for path in sorted(glob.glob(os.path.join(ROOT, windows_path(pattern)))):
            if os.path.isfile(path):
                relative = windows_path(os.path.relpath(path, ROOT))
                if relative not in files:
                    files.append(relative)
    return files


def write_solution(executables):
    entries = []      # (name, relative path, guid, folder, buildable_in_release)
    for relative, folder in PRODUCT_PROJECTS:
        path = os.path.join(VS_DIR, windows_path(relative))
        name = os.path.splitext(os.path.basename(relative))[0]
        entries.append((name, windows_path(os.path.relpath(path, ROOT)), read_guid(path), folder, True))
    for name, path, folder in sorted(executables, key=lambda e: (e[2], e[0])):
        entries.append((name, windows_path(os.path.relpath(path, ROOT)), read_guid(path), folder, False))

    folders = {}

    def folder_guid(folder_path):
        if folder_path not in folders:
            parent = folder_path.rpartition('\\')[0]
            if parent:
                folder_guid(parent)
            folders[folder_path] = project_guid('solution-folder/' + folder_path)
        return folders[folder_path]

    for entry in entries:
        folder_guid(entry[3])
    item_folders = []
    for name, patterns in SOLUTION_ITEM_FOLDERS:
        items = solution_items(patterns)
        if items:
            folder_guid(name)
            item_folders.append((name, items))

    lines = ['', 'Microsoft Visual Studio Solution File, Format Version 12.00', '# Visual Studio Version 18',
             'VisualStudioVersion = 18.0.11205.157', 'MinimumVisualStudioVersion = 10.0.40219.1']
    for name, path, guid, _, _ in entries:
        lines.append('Project("%s") = "%s", "%s", "%s"' % (CPP_PROJECT_TYPE, name, path, guid))
        lines.append('EndProject')
    items_by_folder = dict(item_folders)
    for folder_path, guid in folders.items():
        lines.append('Project("%s") = "%s", "%s", "%s"' % (FOLDER_TYPE, folder_path.rpartition('\\')[2], folder_path.rpartition('\\')[2], guid))
        if folder_path in items_by_folder:
            lines.append('\tProjectSection(SolutionItems) = preProject')
            for item in items_by_folder[folder_path]:
                lines.append('\t\t%s = %s' % (item, item))
            lines.append('\tEndProjectSection')
        lines.append('EndProject')
    lines.append('Global')
    lines.append('\tGlobalSection(SolutionConfigurationPlatforms) = preSolution')
    # Visual Studio opens a solution in its alphabetically first configuration. "Release" builds
    # the application exactly as build.ps1 does; "Test" adds the tests and tools.
    lines.append('\t\tRelease|x64 = Release|x64')
    lines.append('\t\tTest|x64 = Test|x64')
    lines.append('\tEndGlobalSection')
    lines.append('\tGlobalSection(ProjectConfigurationPlatforms) = postSolution')
    for _, _, guid, _, buildable_in_release in entries:
        lines.append('\t\t%s.Release|x64.ActiveCfg = Release|x64' % guid)
        if buildable_in_release:
            lines.append('\t\t%s.Release|x64.Build.0 = Release|x64' % guid)
        lines.append('\t\t%s.Test|x64.ActiveCfg = Release|x64' % guid)
        lines.append('\t\t%s.Test|x64.Build.0 = Release|x64' % guid)
    lines.append('\tEndGlobalSection')
    lines.append('\tGlobalSection(SolutionProperties) = preSolution')
    lines.append('\t\tHideSolutionNode = FALSE')
    lines.append('\tEndGlobalSection')
    lines.append('\tGlobalSection(NestedProjects) = preSolution')
    for name, path, guid, folder, _ in entries:
        lines.append('\t\t%s = %s' % (guid, folders[folder]))
    for folder_path, guid in folders.items():
        parent = folder_path.rpartition('\\')[0]
        if parent:
            lines.append('\t\t%s = %s' % (guid, folders[parent]))
    lines.append('\tEndGlobalSection')
    lines.append('\tGlobalSection(ExtensibilityGlobals) = postSolution')
    lines.append('\t\tSolutionGuid = %s' % project_guid('solution'))
    lines.append('\tEndGlobalSection')
    lines.append('EndGlobal')
    return '\r\n'.join(lines) + '\r\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--build-dir', default=os.path.join(ROOT, 'build', 'cmake'),
                        help='CMake build directory configured with tests and tools (default: build/cmake)')
    parser.add_argument('--check', action='store_true', help='report stale files instead of writing them')
    arguments = parser.parse_args()

    targets, paths = load_codemodel(os.path.abspath(arguments.build_dir))
    outputs = {}
    executables = []
    for target in sorted(targets.values(), key=lambda t: t['name']):
        if target['type'] != 'EXECUTABLE' or target['name'] == 'RigidBodies.App' or target['name'] in SKIPPED_TARGETS:
            continue
        description = describe_executable(target, targets, paths)
        project_path, folder = project_location(description)
        kind = 'tests' if description['name'].endswith('.Tests') else 'tools'
        outputs[project_path] = executable_project(description, project_path, kind)
        executables.append((description['name'], project_path, folder))

    stale = []
    generated_dirs = [os.path.join(VS_DIR, 'tests'), os.path.join(VS_DIR, 'tools')]
    existing = set()
    for directory in generated_dirs:
        for current, _, files in os.walk(directory):
            existing.update(os.path.join(current, f) for f in files if f.endswith('.vcxproj'))
    for path in sorted(existing - set(outputs)):
        stale.append(path)
        if not arguments.check:
            os.remove(path)
    for path, text in outputs.items():
        current = open(path, encoding='utf-8-sig').read() if os.path.isfile(path) else None
        if current is None or current.replace('\r\n', '\n') != text.replace('\r\n', '\n'):
            stale.append(path)
            if not arguments.check:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w', encoding='utf-8', newline='') as handle:
                    handle.write(text)
    solution = write_solution(executables)
    current = open(SOLUTION, encoding='utf-8-sig').read() if os.path.isfile(SOLUTION) else None
    if current is None or current.replace('\r\n', '\n') != solution.replace('\r\n', '\n'):
        stale.append(SOLUTION)
        if not arguments.check:
            with open(SOLUTION, 'w', encoding='utf-8-sig', newline='') as handle:
                handle.write(solution)

    for path in stale:
        print(('stale: ' if arguments.check else 'wrote: ') + os.path.relpath(path, ROOT))
    print(f'{len(executables)} test and tool projects; {len(stale)} file(s) {"out of date" if arguments.check else "updated"}')
    return 1 if arguments.check and stale else 0


if __name__ == '__main__':
    sys.exit(main())
