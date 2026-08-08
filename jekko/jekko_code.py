# Imports

import math
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import threading
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster#new
import queue

# DataFrame

data = {
    'Nummer': [1, 2, 3, 4, 5, 6, 7, 8, 9],
    'Length(m)': [1, 2, 3, 4, 5, 6, 7, 8, 9],
    'Height(m)': [2.6, 3.8, 4.8, 5.8, 6.8, 7.8, 8.8, 9.8, 10.3],
    '0"': ["Impossible", 3200, 2600, 2250, 1500, 1100, 800, 650, 530],
    '10"': ["Impossible", 3050, 2500, 2050, 1400, 1100, 800, 650, 525],
    '20"': ["Impossible", 2900, 2450, 1850, 1350, 1100, 800, 650, 520],
    '30"': ["Impossible", 2750, 2400, 1750, 1300, 1100, 800, 640, "Impossible"],
    '40"': ["Impossible", 2600, 2300, 1650, 1150, 1040, 800, "Impossible", "Impossible"],
    '45"': ["Impossible", 2300, 2150, 1550, 1140, 930, "Impossible", "Impossible", "Impossible"],
    '50"': [3200, 2300, 1950, 1350, 1120, 900, "Impossible", "Impossible", "Impossible"],
    '55"': [2900, 2150, 1700, 1300, 960, "Impossible", "Impossible", "Impossible", "Impossible"],
    '60"': [2650, 2090, 1530, 1180, "Impossible", "Impossible", "Impossible", "Impossible", "Impossible"],
    '65"': [2400, 1880, 1410, "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible"],
    '70"': [2300, 1550, "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible"],
    '80"': [1450, "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible", "Impossible"],
}

df = pd.DataFrame(data)

# Class Defs

class JekkoCraneController(Node):
    def __init__(self, result_queue):
        super().__init__('jekko_crane_controller_node')
        self.e = self.create_publisher(String, 'chosen_configuration', 10)
        self.get_logger().info('JekkoCraneController has been initialized.')
        timer_period = 0.5  # saniye
        self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(String, 'jekko_camera', self.subscriber_callback, 10)
        self.x = 0.00
        self.y = 0.00
        self.weight = 0.00
        self.height = 0.00
        self.result = None
        self.result_queue = result_queue
    
    def get_result(self):
        return self.result

    def publisher_callback(self, publisher, msg_ros):  

        try:
            msg = String()
            msg.data = msg_ros
            publisher.publish(msg)
            self.result_queue.put(msg_ros)

        except Exception as e:
            print(f'Error publishing message: {e}')

    def subscriber_callback(self, msg):
        try:
            received_data = msg.data.split(',')
            if received_data:
                self.x = float(received_data[2]) / 50 
                self.y = float(received_data[3]) / 50
                self.height = float(received_data[0])
                self.weight = float(received_data[1])
                

                self.get_logger().warn(f"{self.x}, {self.y}, {self.height}, {self.weight}")
                print(self.x, self.y)
              
                result = self.get_value_with_length_consideration(self.x, self.y, self.height, self.weight)
                result_str = str(result)
                self.publisher_callback(self.e, result_str)
            elif received_data is None:
                print("No message recieved yet.")
                return
            else:
                self.get_logger().warn('Invalid data format received from "jekko_camera"')

        except Exception as e:
            self.get_logger().error(f'Error receiving message: {e}')

    
    def timer_callback(self):
        result = self.get_value_with_length_consideration(self.x, self.y, self.height, self.weight)
        result_str = str(result)
        self.publisher_callback(self.e, result_str)

    
    def calculate_triangle_sides(self, adjacent_length, alpha_degrees):
        alpha_radians = math.radians(alpha_degrees)
        max_attempts = 10

        for _ in range(max_attempts):
            # Check if cosine of alpha_radians is zero to avoid division by zero
            if math.isclose(math.cos(alpha_radians), 0.0, abs_tol=1e-9):
                self.get_logger().warning("Cosine of alpha is zero. Retrying calculation.")
            else:
                try:
                    hypotenuse = adjacent_length / math.cos(alpha_radians)
                    opposite_side = adjacent_length * math.tan(alpha_radians)
                    return hypotenuse, opposite_side
                except ZeroDivisionError:
                    self.get_logger().error("Error: Division by zero in calculate_triangle_sides.")
                except Exception as e:
                    self.get_logger().error(f"Error in calculate_triangle_sides: {e}")

        self.get_logger().error(f"Unable to calculate triangle sides after {max_attempts} attempts.")
        return float('inf'), float('inf')
    


    def get_value_with_length_consideration(self, x, y, height, weight):
        material_distance = math.sqrt(x**2 + y**2)
        # print(f"Material Distance: {material_distance}")
        if material_distance != 0:
            theta = math.degrees(math.asin(y / material_distance))
            theta_round = round(theta, 2)
            # print(f"Material Theta: {theta_round}")
        
            rounded_material_distance = math.ceil(material_distance)


            if rounded_material_distance > 9.5:
                # print("The material is too far away. Please change the location.")
                return "The material is too far away."
            else:
                min_boom_length = float('inf')
                closest_index = (df['Length(m)'] - rounded_material_distance).abs().idxmin()
                weights = df.loc[closest_index, '10"':].tolist()

                if weights:

                    for column, weightt in zip(df.columns[3:], weights):
                        angle = int(column.replace('"', ''))
                        
                        if weightt != "Impossible" and weightt >= weight:
                            boom_length, z = self.calculate_triangle_sides(material_distance, angle)

                            boom_length_round = round(boom_length, 6)
                            z_round = round(z, 6)
                            material_distance_round = round(material_distance, 6)

                            if z + 2 >= height and boom_length < min_boom_length:
                                # print(f"Boom length at {column}\" is viable with boom length of {boom_length_round} and total height of {z_round + 2}")
                                chosen_configuration = f"Distance to Material: {material_distance_round}, Material Theta Degrees: {theta_round}, Alpha Degrees: {angle}, Boom Length: {boom_length_round}, Z: {z_round + 2}, Height to Material: {z_round + 2 - height}"
                                min_boom_length = boom_length
                            # elif z + 2 > height:
                                # print(f"Boom length at {column}\" is possible but won't be chosen.")
                            # else:
                                # print(f"Boom length at {column}\" is not possible due to the height of the material.")
                        # else:
                            # print(f"Weight cannot be carried at {column}\"")

                    if chosen_configuration is not None:
                        # print(f"Chosen Configuration: {chosen_configuration}")
                        self.result = chosen_configuration
                    
                    else:
                        self.result = "No viable configuration found."
                    return self.result
                        

                        
                else:
                    # print("All weights are marked as 'Impossible.'")
                    return "All weights are marked as 'Impossible.'"


class JointPublisher(Node):
    def __init__(self, name, result_queue):
        super().__init__(name)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.result_queue = result_queue

        self.joint = JointState()
        self.joint.header.frame_id = 'base_link'
        self.joint.name = ['base_rotatory',
                           'rotatory_arm1',
                           "arm1_arm2",
                           "arm2_arm3",
                           "arm3_arm4",
                           "arm4_arm5",
                           "arm5_hook_rotatory",
                           "arm5_hook_prismatic"]

        self.base_rotatory = 0.0
        self.rotatory_arm_1 = 0.0
        self.arm1_arm2 = 0.0
        self.arm2_arm3 = 0.0
        self.arm3_arm4 = 0.0
        self.arm4_arm5 = 0.0
        self.arm5_hook_rotatory = 0.0
        self.arm5_hook_prismatic = 0.0

        self.timer = self.create_timer(1, self.publish_joint_positions)

        self.get_logger().info(f"{name} is now on ...")

    def publish_joint_positions(self):
        self.joint.header.stamp = self.get_clock().now().to_msg()

        theta = 0.0
        alpha = 0.0
        boom_length = 0.0
        self.height_to_material = 0.0

        if not self.result_queue.empty() and self.result_queue != 'None':
            result_from_jekko = self.result_queue.get()
            self.get_logger().warning(result_from_jekko)
            if result_from_jekko != "None" and result_from_jekko != None:
                if result_from_jekko == "No viable configuration found.":
                    theta = 0.0
                    alpha = 0.0
                    boom_length = 0.0
                    self.height_to_material = 0.0

                elif result_from_jekko is not None:
                    list_values = result_from_jekko.split(",")  # Split the string into a list
                    
                    values = [value.split(':')[-1].strip() for value in list_values]
                    material_distance = float(values[0])
                    theta = round(float(values[1]),2)
                    print("theta:",theta)
                    alpha = round(float(values[2]),2)
                    print("alpha:", alpha)
                    boom_length = round(float(values[3]),2)
                    z_value = float(values[4])
                    print("z:",z_value)
                    self.height_to_material = round(float(values[5]),2)
                    print("height to material:",self.height_to_material)

        else:
            self.get_logger().warning("No valid result obtained from JekkoCraneController.")
            theta = 0.0
            alpha = 0.0
            boom_length = 0.0
            self.height_to_material = 0.0
        


        # Set the joint positions
        self.base_rotatory = np.radians(theta)
        self.rotatory_arm_1 = np.radians(alpha)
        self.arm5_hook_rotatory = np.radians(alpha)
        self.arm5_hook_prismatic = self.height_to_material
        
        
        if boom_length:
            self.arm1_arm2 = 0.00
            self.arm2_arm3 = 0.00
            self.arm3_arm4 = 0.00
            self.arm4_arm5 = 0.00

            if boom_length >= 9.50:
                self.arm1_arm2 = 1.70
                self.arm2_arm3 = 2.00
                self.arm3_arm4 = 2.00
                self.arm4_arm5 = 2.00
            elif boom_length > 7.50:
                self.arm1_arm2 = 1.90
                self.arm2_arm3 = 2.00
                self.arm3_arm4 = 2.00
                self.arm4_arm5 = boom_length - 7.50
            elif boom_length > 5.50:
                self.arm1_arm2 = 1.90
                self.arm2_arm3 = 2.00
                self.arm3_arm4 = boom_length - 5.50
                self.arm4_arm5 = 0.00
            elif boom_length > 3.50:
                self.arm1_arm2 = 1.90
                self.arm2_arm3 = boom_length - 3.50
                self.arm3_arm4 = 0.00
                self.arm4_arm5 = 0.00
            elif boom_length > 1.90:
                self.arm1_arm2 = boom_length - 1.90
                self.arm2_arm3 = 0.00
                self.arm3_arm4 = 0.00
                self.arm4_arm5 = 0.00

            
                                

        # Set joint positions in the JointState message
        self.joint.position = [
            self.base_rotatory,
            self.rotatory_arm_1,  # Set the angle calculated based on theta
            self.arm1_arm2,
            self.arm2_arm3,
            self.arm3_arm4,
            self.arm4_arm5,
            self.arm5_hook_rotatory,
            self.arm5_hook_prismatic
        ]

        # Publish the JointState message
        self.joint_state_publisher.publish(self.joint)
        self.get_logger().info(f"Joint publisher published, {self.base_rotatory}")


# Main Function

def main(args=None):
    rclpy.init(args=args)
    
    result_queue = queue.Queue()

    jekko_crane_controller = JekkoCraneController(result_queue)
    joint_publisher = JointPublisher("jekko", result_queue)

    jekko_executor = rclpy.executors.SingleThreadedExecutor()
    joint_executor = rclpy.executors.SingleThreadedExecutor()

    jekko_executor.add_node(jekko_crane_controller)
    joint_executor.add_node(joint_publisher)

    jekko_thread = threading.Thread(target=lambda: jekko_executor.spin())
    joint_thread = threading.Thread(target=lambda: joint_executor.spin())

    jekko_thread.start()
    joint_thread.start()

    jekko_thread.join()
    joint_thread.join()

    joint_publisher.destroy_node()
    jekko_crane_controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()