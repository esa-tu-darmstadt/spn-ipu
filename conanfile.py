from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import apply_conandata_patches, copy, export_conandata_patches, get, rm, rmdir
import os


required_conan_version = ">=2.0.9"


class GrapheneConan(ConanFile):
    name = "spnc-ipu"
    version = "0.1.0"
    author = "ESA TU Darmstadt"
    description = "Sum-Product Network Compiler for IPUs"
    topics = ("simulation", "finite-volume", "ipu", "graphcore")
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "graphene_as_submodule": [True, False],
        "ipu_arch_ipu1": [True, False],
        "ipu_arch_ipu2": [True, False],
        "ipu_arch_ipu21": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "graphene_as_submodule": True,
        "ipu_arch_ipu1": False,
        "ipu_arch_ipu2": True,
        "ipu_arch_ipu21": True,
    }
    generators = "CMakeDeps", "CMakeToolchain"
    implements = ["auto_shared_fpic"]


    def export_sources(self):
        copy(self, "CMakeLists.txt", self.recipe_folder, self.export_sources_folder)
        copy(self, "cmake/*",          self.recipe_folder, self.export_sources_folder)
        copy(self, "apps/*",          self.recipe_folder, self.export_sources_folder)
        copy(self, "librheos/*",          self.recipe_folder, self.export_sources_folder)

    def layout(self):
        cmake_layout(self, src_folder=".")

    def requirements(self):
        # We can use graphene either as a dependency or as a submodule. This setting here must be consistent with the
        # CMakeLists.txt file in the root directory!
        if self.options.graphene_as_submodule:
            # If we use graphene as a submodule, we need to add its dependencies here:
            # TODO: Read the dependencies from the graphene conanfile.py somehow?
            self.requires("metis/5.2.1")
            self.requires("twofloat/0.2.0")
            self.requires("nlohmann_json/3.9.1") # Same version as in Poplar SDK
            self.requires("fast_matrix_market/1.7.6")
            self.requires("fmt/11.1.3")
            self.requires("gtest/1.16.0")
            self.requires("cli11/2.3.2")
            self.requires("spdlog/1.15.1")
        else:
            self.requires("graphene/0.1.0")
            self.requires("gtest/1.16.0")
            self.requires("fmt/11.1.3")
            self.requires("cli11/2.3.2")
            self.requires("spdlog/1.15.1")
            self.requires("nlohmann_json/3.9.1")

    
        # Our own dependencies:
        self.requires("capnproto/1.1.0")
        self.requires("dagp/1.0")

    def build(self):
        cmake = CMake(self)
        
        configVars = { 
            "USE_GRAPHENE_SUBMODULE": self.options.graphene_as_submodule
        }
        
        if self.options.graphene_as_submodule:
            # Build list of enabled IPU architectures from options
            enabled_ipu_archs = []
            if self.options.ipu_arch_ipu1:
                enabled_ipu_archs.append("ipu1")
            if self.options.ipu_arch_ipu2:
                enabled_ipu_archs.append("ipu2")
            if self.options.ipu_arch_ipu21:
                enabled_ipu_archs.append("ipu21")
            
            # Make sure at least one architecture is enabled
            if not enabled_ipu_archs:
                self.output.warning("No IPU architectures enabled. Defaulting to ipu2.")
                enabled_ipu_archs = ["ipu2"]
            
            # Join the architectures with commas for the CMake variable
            ipu_archs_str = ";".join(enabled_ipu_archs)
            configVars["POPLIBS_ENABLED_IPU_ARCH_NAMES"] = ipu_archs_str

        cmake.configure(variables=configVars)
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()


    def package_info(self):
        pass