YAML-CPP

		sudo apt-get install libyaml-cpp-dev

SERIAL-CPP
		sudo apt-get install libboost-all-dev
		sudo apt-get install ros-noetic-serial


PCAN_BASIC-c
		-------------------driver
		sudo apt update
		sudo apt install libpopt-dev
		tar -xzf peak-linux-driver-8.18.0.tar.gz
		cd peak-linux-driver-8.18.0
		sudo make clean all
		sudo make install
		sudo modprobe pcan

		------------------- lib
		tar -xzf PCAN-Basic_Linux-4.8.0.5.tar.gz
 		cd PCAN-Basic_Linux-4.8.0.5/libpcanbasic/pcanbasic
		sudo make clean
		sudo make
		sudo make install


ArenaAPI
		tar xvzf ArenaSDK_v0.1.90_Linux_x64
		cd ./ArenaSDK_v0.1.90_Linux_x64/ArenaSDK_Linux_x64/
		sudo sh Arena_SDK_Linux_x64.conf
		sudo reboot

		tar -xvzf Lucid_Broadcom_Driver_Package_v*.tar.gz
		cd Lucid_Broadcom_Driver_Package
		sudo ./install_broadcom_driver.sh

		------------------------------------------------------------------
		이때, 아래와 같은 에러가 발생 가능하지만 무시하면 됩니다.
		Interface -w not found
		There was an error during the driver installation. Error code: 0
		Check log for specific details on the cause of the failure.
		-------------------------------------------------------------------

Faiss
		git clone https://github.com/facebookresearch/faiss
	이후, INSTALL.md 의 Building from source 따라 진행 
	또한, CMakeLists.txt에서 option(FAISS_ENABLE_C_API "Build C API." ON) 필요
	만약 이미 cmake한 build 폴더가 존재할 경우
		make -C build install


KVASER related
	- https://www.kvaser.com/canlib-webhelp/section_install_linux.html



	kvaser Driver:

		sudo apt-get install build-essential 

		sudo apt-get install pkg-config 

		wget --content-disposition "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest"

		tar xvzf linuxcan.tar.gz

		cd linuxcan

		make

		sudo make install 

		sudo make load 
		
		
	kvaser canlib

		wget --content-disposition "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130981966&utm_status=latest"

		tar xvzf kvlibsdk.tar.gz
		
		cd kvlibsdk
		
		make
		
		make check 
		
		sudo make install
		
Ipopt related(Solver)
	
	Ipopt dependencies
		
		sudo apt-get install build-essential gfortran liblapack-dev libmetis-dev libopenblas-dev
		
	Ipopt:
		cd
		
		mkdir -p  ~/CoinIpopt && cd ~/CoinIpopt
		
		wget https://raw.githubusercontent.com/coin-or/coinbrew/master/coinbrew

		chmod u+x coinbrew

		./coinbrew Ipopt
		
		1
		
		y
		
		mkdir install
		
		./coinbrew build Ipopt --prefix=install --test --no-prompt --verbosity=3
		
		dependencies에 있는 coinhsl 복사 후 ~/CoinIpopt/ThirdParty/HSL/ 에 붙혀넣기
		
		cd ~/CoinIpopt
		
		./coinbrew build Ipopt --prefix=install --test --no-prompt --verbosity=3
		
		./coinbrew install Ipopt --no-prompt
		
		cd ~/CoinIpopt/install/lib
		
		ln -s libcoinhsl.so libhsl.so
		
		gedit ~/.bashrc
		
		bashrc 제일 밑단에 복사 후 추가
		-----------------------------------------------------
		export IPOPT_DIR=~/CoinIpopt/install
		export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig
		export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib

		# ipopt this is required for casadi
		export PATH=${PATH}:${IPOPT_DIR}/lib

		# this may speed up ipopt 
		export OMP_NUM_THREADS=1
		----------------------------------------------------
		
		source ~/.bashrc

		
casadi related(QP Solve lib)

	casadi dependencies
	
		sudo apt install gfortran liblapack-dev pkg-config --install-recommends
		
		sudo apt install swig
	
	casadi:
		cd
		
		git clone https://github.com/casadi/casadi.git -b main casadi // casadi 미설치 일때만
		
		
		기존 casadi 설치 되어 있으면 casadi 폴더 내 build 삭제
		
		cd casadi
		
		mkdir -p build
		
		cd build
		
		cmake  -DWITH_IPOPT:BOOL=ON -DWITH_HSL:BOOL=ON -DINCLUDE_PREFIX:PATH=include -DCMAKE_PREFIX:PATH=lib/cmake/casadi -DLIB_PREFIX:PATH=lib -DBIN_PREFIX:PATH=bin ..
		
		make
		
		sudo make install
		
		
		
		
		
		
	curl: 
		sudo apt update
		sudo apt install libcurl4-openssl-dev
		
	rapidjson:
		sudo apt-get install rapidjson-dev
		
	SOIL:
		sudo apt-get install libsoil-dev
	
