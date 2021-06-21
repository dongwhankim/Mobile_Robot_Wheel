add_executable(test
        Controller.cpp
        Communication.cpp
        )
        

        target_link_libraries(test pthread)