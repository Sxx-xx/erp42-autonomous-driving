#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
from v2x_decoder import J2735Decoder
import v2x_msgs.msg

def spat_dict_to_msg(decoded_spat):
    spat_msg = v2x_msgs.msg.Spat()

    for intersection_data in decoded_spat['intersections']:
        interchange_msg = v2x_msgs.msg.Interchange()
        id_msg = v2x_msgs.msg.Id(region=intersection_data['id']['region'], id=intersection_data['id']['id'])
        interchange_msg.id = id_msg

        for state_data in intersection_data['states']:
            state_msg = v2x_msgs.msg.State()
            state_msg.movementName = state_data['movementName']
            state_msg.signalGroup = state_data['signalGroup']

            state_time_speed_msg = v2x_msgs.msg.State_time_speed()

            if ( 'eventState' in state_data['state-time-speed'][0].keys()) and ('timing' in state_data['state-time-speed'][0].keys()) :  
                state_time_speed_msg.event_state = state_data['state-time-speed'][0]['eventState']
                state_time_speed_msg.timing.minEndTime = state_data['state-time-speed'][0]['timing']['minEndTime']


            
                if 'maxEndTime' in state_data['state-time-speed'][0]['timing']:
                    state_time_speed_msg.timing.maxEndTime = state_data['state-time-speed'][0]['timing']['maxEndTime']
                else:   
                    state_time_speed_msg.timing.maxEndTime = -1  # -1 stands for "not set"

                state_msg.state_time_speed = state_time_speed_msg
                interchange_msg.states.append(state_msg)
            else:
                continue
                # TODO: Need to handle exception case
                # print(interchange_msg.id)
                # print(state_msg)
                # print("execption")

        spat_msg.interchanges.append(interchange_msg)

    return spat_msg

def udp_listener():
    rospy.init_node('v2x_publisher_node', anonymous=True)
    pub = rospy.Publisher('v2x_data', v2x_msgs.msg.Spat, queue_size=10)
    
    # Set up UDP socket
    UDP_IP = "192.168.1.3"
    UDP_PORT = 60000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    rospy.loginfo("UDP listener node is now listening on port %d", UDP_PORT)

    # Set up J2735 decoder
    decoder_instance = J2735Decoder()
    
    while not rospy.is_shutdown():
        received_data, addr = sock.recvfrom(1500)
        hex_data = "".join("{:02x}".format(byte) for byte in received_data)

        # Remove the first 16 bytes (header) and the last 2 bytes (tail)
        hex_data = hex_data[32:-4]

        if hex_data[:4] == "0013":  # If data is SPAT
            bytes_data = bytes.fromhex(hex_data)
            decoded_spat = decoder_instance.spat_decoder(bytes_data)

            spat_msg = spat_dict_to_msg(decoded_spat)
            pub.publish(spat_msg)

        # elif hex_data[:4] == "0012":  # If data is MAP
        #     bytes_data = bytes.fromhex(hex_data)
        #     decoded_map = decoder_instance.map_decoder(bytes_data)
        #     print(decoded_map)
        #     pub.publish(str(decoded_map))

def sudo_udp_listener():
    rospy.init_node('v2x_publisher_node', anonymous=True)
    pub = rospy.Publisher('v2x_data', v2x_msgs.msg.Spat, queue_size=10)

    rate = rospy.Rate(10)  # 1Hz 주기로 실행하려고 설정

    # Set up J2735 decoder
    decoder_instance = J2735Decoder()
    
    while not rospy.is_shutdown():
        hex_data = "001380ad22db3260c22d486c18b064c000200c800c8041c143d02a75290082000000103991635402040000002054ea520304000000207322c6a8080800000040a9d4a40a0800000040e6458d5018100000008153a9481c1000000081cc8b1aa0402000000102a7529048200000010399163540a040000002054ea520b04600064207322c6a8180800000040a9d4a41a0800000040e6458d503810c004ec8153a9483c1180032081cc8b1aa0802180039809209"

        bytes_data = bytes.fromhex(hex_data)
        decoded_spat = decoder_instance.spat_decoder(bytes_data)
        spat_msg = spat_dict_to_msg(decoded_spat)
        pub.publish(spat_msg)

        rate.sleep()  # 설정한 주기만큼 대기


if __name__ == '__main__':
    try:
        udp_listener()
        # sudo_udp_listener()
    except rospy.ROSInterruptException:
        pass