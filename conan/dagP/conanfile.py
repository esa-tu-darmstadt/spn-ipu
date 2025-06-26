from conan import ConanFile
from conan.tools.files import get, copy, save, replace_in_file, chdir, patch
from conan.tools.scons import SConsDeps
from conan.tools.layout import basic_layout
from conan.tools.build import check_min_cppstd
from conan.errors import ConanInvalidConfiguration
import os
import textwrap

from pprint import pprint

class DagPConan(ConanFile):
    name = "dagp"
    version = "1.0"
    license = "lgpl-v3" 
    author = "TDA Lab at Georgia Tech <tdalab@cc.gatech.edu>"
    url = "https://github.com/GT-TDAlab/dagP"
    homepage = "http://tda.gatech.edu"
    description = "Multilevel Algorithms for Acyclic Partitioning of Directed Acyclic Graphs"
    topics = ("graph", "partitioning", "dag", "scheduling")
    
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_metis": [True, False],
        "with_scotch": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_metis": True,
        "with_scotch": False,
    }
    exports_sources = "*.patch"

    # Binary configuration
    package_type = "library"
    
    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")
            
    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")
            
    def layout(self):
        basic_layout(self, src_folder="src")
        
    def requirements(self):
        if self.options.with_metis:
            self.requires("metis/5.2.1")
        if self.options.with_scotch:
            self.requires("scotch/6.1.0")
            
    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, "11")
            
    def build_requirements(self):
        self.tool_requires("scons/4.6.0")
        
    def source(self):
        # Download from GitHub
        get(self, "https://github.com/GT-TDAlab/dagP/archive/refs/heads/master.zip",
            destination=self.source_folder, strip_root=True)

        # Apply the patch 
        patch_file = os.path.join(self.export_sources_folder, "sconstruct.patch")
        patch(self, patch_file=patch_file)
        
            
    def generate(self):
        # Generate dependencies information for SCons
        deps = SConsDeps(self)
        deps.generate()
        
    def build(self): 
        # Copy source files to build folder
        copy(self, "*", src=self.source_folder, dst=self.build_folder, keep_path=True)
        
        # Generate config.py for SCons build
        config_content = []
    
        config_content.append(f"CC = '{self.buildenv.vars(self)['CC']}'")
        config_content.append(f"CXX = '{self.buildenv.vars(self)['CXX']}'")
        config_content.append(f"LINK = '{self.buildenv.vars(self)['CXX']}'")
        
        config_content.append(f"debug = {1 if self.settings.build_type == 'Debug' else 0}")
        if self.settings.build_type == "RelWithDebInfo":
            config_content.append("CCFLAGS = '-g'")
            
            
        if self.options.with_metis:
            config_content.append("metis = 1")
            config_content.append("LIBS = ['metis']")
        save(self, os.path.join(self.build_folder, "config.py"), "\n".join(config_content))


        
        self.run(f"scons -C {self.build_folder}")
        
    def package(self):
        # Copy headers
        copy(self, "dgraph.h", 
             src=os.path.join(self.build_folder, "src", "common"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=False)
        copy(self, "option.h", 
             src=os.path.join(self.build_folder, "src", "common"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=False)
        copy(self, "dagP.h", 
             src=os.path.join(self.build_folder, "src", "recBisection"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=False)
             
        # Copy libraries
        copy(self, "*.a",
             src=os.path.join(self.build_folder, "lib"),
             dst=os.path.join(self.package_folder, "lib"),
             keep_path=False)
        copy(self, "*.so",
             src=os.path.join(self.build_folder, "lib"),
             dst=os.path.join(self.package_folder, "lib"),
             keep_path=False)
        copy(self, "*.dylib",
             src=os.path.join(self.build_folder, "lib"),
             dst=os.path.join(self.package_folder, "lib"),
             keep_path=False)
             
        # Copy executables (optional)
        copy(self, "*",
             src=os.path.join(self.build_folder, "exe"),
             dst=os.path.join(self.package_folder, "bin"),
             keep_path=False,
             excludes="*.o")
             
        # Copy license
        copy(self, "LICENSE*",
             src=self.build_folder,
             dst=os.path.join(self.package_folder, "licenses"),
             keep_path=False)
             
    def package_info(self):
        self.cpp_info.libs = ["dagp"]
        
        # Set library directories
        self.cpp_info.includedirs = ["include", "include"]
        self.cpp_info.libdirs = ["lib"]
        self.cpp_info.bindirs = ["bin"]
        
        self.cpp_info.set_property("cmake_file_name", "dagP")
        self.cpp_info.set_property("cmake_target_name", "dagP::dagP")