FROM mcr.microsoft.com/devcontainers/cpp:1-debian-12

# Install newer cmake version
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.31.6"
COPY ./reinstall-cmake.sh /tmp/
RUN chmod +x /tmp/reinstall-cmake.sh && sudo /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE} \
    && rm -f /tmp/reinstall-cmake.sh

# Install old llvm packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get install -y clang lld

# Install latest llvm packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y lsb-release wget software-properties-common gnupg
RUN bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"

# Install poplar to /poplar
WORKDIR /poplar
RUN curl --location --request GET 'https://downloads.graphcore.ai/direct?package=poplar-poplar_sdk_ubuntu_20_04_3.4.0_69d9d03fd8-3.4.0&file=poplar_sdk-ubuntu_20_04-3.4.0-69d9d03fd8.tar.gz' --output 'poplar_sdk-ubuntu_20_04-3.4.0-69d9d03fd8.tar.gz' \
    && tar -xvf poplar_sdk-ubuntu_20_04-3.4.0-69d9d03fd8.tar.gz \
    && rm poplar_sdk-ubuntu_20_04-3.4.0-69d9d03fd8.tar.gz

# Install dependencies of poplar
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y \
    git \
    python3 \
    python3-pip \
    python3-distutils \
    make \
    libssl-dev \
    zlib1g-dev \
    rapidjson-dev \
    libboost-dev \
    libre2-dev \
    librdmacm-dev

WORKDIR /home/vscode
RUN echo "source /poplar/poplar_sdk-ubuntu_20_04-3.4.0+1507-69d9d03fd8/enable" >> /home/vscode/.bashrc

# Required by poplar, but not provided by Debian 12 anymore: libssl1.1
WORKDIR /tmp
RUN wget http://security.debian.org/debian-security/pool/updates/main/o/openssl/libssl1.1_1.1.1w-0+deb11u3_amd64.deb \
    && dpkg -i libssl1.1_1.1.1w-0+deb11u3_amd64.deb \
    && rm libssl1.1_1.1.1w-0+deb11u3_amd64.deb

# Build and install or-tools required for scheduling SPNs
COPY ./build-ortools.sh /tmp/
RUN chmod +x /tmp/build-ortools.sh && sudo /tmp/build-ortools.sh \
    && rm -f /tmp/build-ortools.sh

# Install Gurobi
WORKDIR /opt
RUN wget https://packages.gurobi.com/12.0/gurobi12.0.1_linux64.tar.gz && \
    tar -xf gurobi12.0.1_linux64.tar.gz && \
    rm gurobi12.0.1_linux64.tar.gz
ENV LD_LIBRARY_PATH="/opt/gurobi1201/linux64/lib:$LD_LIBRARY_PATH"
ENV PATH="/opt/gurobi1201/linux64/bin:$PATH"
ENV GUROBI_HOME="/opt/gurobi1201/linux64"
RUN echo "TOKENSERVER=lic1.hrz.tu-darmstadt.de" > /opt/gurobi1201/gurobi.lic
ENV GRB_LICENSE_FILE="/opt/gurobi1201/gurobi.lic"

RUN apt-get update && apt-get -y install graphviz

# Install conan
RUN apt-get update && apt-get install -y pipx
USER vscode
RUN pipx install conan
RUN mkdir -p /home/vscode/.conan2/profiles
COPY --chown=vscode:vscode ./conan-profiles/clang-debug /home/vscode/.conan2/profiles/clang-debug
COPY --chown=vscode:vscode ./conan-profiles/clang-release /home/vscode/.conan2/profiles/clang-release
RUN ln -sf /home/vscode/.conan2/profiles/clang-release /home/vscode/.conan2/profiles/default