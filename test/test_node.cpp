#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp_simple/srv/add_two_ints.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

using namespace std::chrono_literals;


class PubTest (test_services_client, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_ONE(PubTest(test_services_client, RMW_IMPLEMENTATION), test_add_noreqid) {
  auto node = rclcpp::Node::make_shared("test_services_client_no_reqid");

  auto client = node->create_client<test_rclcpp_simple::srv::AddTwoInts>("add_two_ints_noreqid");
  auto request = std::make_shared<test_rclcpp_simple::srv::AddTwoInts::Request>();
  request->a = 1;
  request->b = 2;

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(3, result.get()->sum);
}



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}