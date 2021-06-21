# �䱸 CMake �ּ� ����
CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
# ������Ʈ �̸� �� ����
PROJECT ( "LALO" )
 
 
# ���� ����(Configuration) �� �������� Makefile ���� ���� & # ���� �����Ϸ�

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-m64")
set(THREADS_PREFER_PTHREAD_FLAG ON)
add_definitions("-Wall -std=c++14")
 
# ���� ��� ���� Include ���丮 (-I)
INCLUDE_DIRECTORIES (include )#/usr/local/include/eigen3 /lib/x86_64-linux-gnu )
 
# ���� ������ �ɼ�, ��ũ �ɼ�
ADD_COMPILE_OPTIONS ( -g -Wall )
SET ( CMAKE_EXE_LINKER_FLAGS "-static -Wl,--gc-sections" )
add_subdirectory(src)
 
# "Debug" ���� ���� ������ �ɼ�, ��ũ �ɼ�
#SET ( CMAKE_C_FLAGS_DEBUG "-DDEBUG -DC_FLAGS" )
#SET ( CMAKE_EXE_LINKER_FLAGS_DEBUG "-DDEBUG -DLINKER_FLAGS" )
 
# "Release" ���� ���� ������ �ɼ�, ��ũ �ɼ�
#SET ( CMAKE_C_FLAGS_RELEASE "-DRELEASE -DC_FLAGS" )
#SET ( CMAKE_EXE_LINKER_FLAGS_RELEASE "-DRELEASE -DLINKER_FLAGS" )
 
# ��� ���丮
SET ( CMAKE_RUNTIME_OUTPUT_DIRECTORY /build )
SET ( CMAKE_LIBRARY_OUTPUT_DIRECTORY /build )
SET ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY /build )

set(THREADS_PREFER_PTHREAD_FLAG ON)