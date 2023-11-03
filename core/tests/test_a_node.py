import pytest
import rclpy
from std_msgs.msg import Int64
from core.a_node import is_prime, calculate_n_primes, ANode

@pytest.fixture
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_is_prime():
    assert is_prime(2) == True
    assert is_prime(3) == True
    assert is_prime(4) == False
    assert is_prime(17) == True
    assert is_prime(25) == False

def test_calculate_n_primes():
    primes = calculate_n_primes(5)
    assert primes == [2, 3, 5, 7, 11]

def test_anode(rclpy_setup_teardown):
    node = ANode()
    node.calculate_next_callback(Int64(data=5))
    assert node.msg_count == 1

    node.calculate_next_callback(Int64(data=10))
    assert node.msg_count == 2
