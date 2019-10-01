import glob, os 


library_name = "pathetic"
standard = "17"
source_dir = ["src"]
subdirs = ["control", "kinematics", "math", "path", "profile"]
sources = []

cwd = os.getcwd()

base = f"""
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD {standard})

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project({library_name})
\n
"""

def get_sources_in_dir(source_dir, subdirs):
    os.chdir(source_dir)
    for subdir in subdirs:
        os.chdir(subdir)
        for file in glob.glob("*.cc"):
            sources.append("\t" + source_dir + "/" + subdir + "/" + file)
        os.chdir(cwd + "/" + source_dir)

get_sources_in_dir(source_dir[0], subdirs)
sources_string = '\n'.join(sources)

lib_str = f"""
add_library({library_name} SHARED
{sources_string}
)
"""

headers_str = f"""
target_include_directories({library_name}
    PRIVATE 
        ${{PROJECT_SOURCE_DIR}}/include
)
"""
os.chdir(cwd)
with open('CMakeLists.txt', 'w+') as f:
    f.write(base + lib_str + headers_str)

