
add_executable(HelloTest HelloTest.cc)
target_link_libraries(HelloTest
        ${THIRD_PARTY_LIBS}
        primerSlam)

add_executable(byebyeTest byebyeTest.cc)
target_link_libraries(byebyeTest
        ${THIRD_PARTY_LIBS}
        primerSlam)

add_executable(trackingTest trackingTest.cc)
target_link_libraries(trackingTest
        ${THIRD_PARTY_LIBS}
        primerSlam)