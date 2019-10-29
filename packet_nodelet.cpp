//
// Created by parsian-ai on 10/6/17.
//

#include <parsian_communication/packet_generate/packet_nodelet.h>
#include <parsian_msgs/parsian_world_model.h>
PLUGINLIB_EXPORT_CLASS(parsian_packet::PacketNodelet, nodelet::Nodelet)

using namespace parsian_packet;


void PacketNodelet::onInit() {


    ros::NodeHandle& n = getNodeHandle();
    ros::NodeHandle& private_n = getPrivateNodeHandle();
    timer = n.createTimer(ros::Duration(.062), boost::bind(&PacketNodelet::timerCb, this, _1));

    drawer = new Drawer();

    drawPub    = n.advertise<parsian_msgs::parsian_draws>("draws", 1000);
    packetPub  = n.advertise<parsian_msgs::parsian_packets>("packets", 1000);

    for (int i = 0 ; i < _MAX_ROBOT_NUM; i++) {
        robotPacketSub[i]   = n.subscribe(QString("/agent_%1/command").arg(i).toStdString(), 100, &PacketNodelet::callBack, this);
    }
    wmSub  = n.subscribe("world_model" , 100, &PacketNodelet::syncData, this);

    for (auto &robotPacket : robotPackets) {
        for (unsigned char &j : robotPacket) {
            j = 255;
        }
    }
    visionCounter = 0;

}

void PacketNodelet::callBack(const parsian_msgs::parsian_robot_commandConstPtr &_packet) {

    parsian_msgs::parsian_robot_command temp = *_packet;

    ROS_INFO("miad inja");
    unsigned char outputBuffer[_ROBOT_PACKET_SIZE] = {0};
    unsigned char robotId = _packet->robot_id;
    int kickNumber = static_cast<int>(round(_packet->kickSpeed));

    outputBuffer[0] = 0x99;
    outputBuffer[1] = robotId & static_cast<unsigned char>(0x0F);
    if (_packet->roller_speed != 0) {
        outputBuffer[1] = static_cast<unsigned char>(outputBuffer[1] | ((_packet->roller_speed & 0x07) << 4));
    }
    outputBuffer[2] = static_cast<unsigned char>(kickNumber & 0x7F);
    outputBuffer[3] = static_cast<unsigned char>((kickNumber >> 7) & 0x07);

    int vforwardI = static_cast<int>(floor(_packet->vel_F * 487.0));
    int vnormalI  = static_cast<int>(floor(_packet->vel_N * 487.0));
    int vangularI  = static_cast<int>((_packet->vel_w) * ((double)256.0 / (double) 360.0));

    if (vforwardI > 2047) {
        vforwardI = 2047;
    }
    if (vforwardI < -2047) {
        vforwardI = -2047;
    }

    if (vnormalI > 2047) {
        vnormalI = 2047;
    }
    if (vnormalI < -2047) {
        vnormalI = -2047;
    }

    if (vangularI > 2047) {
        vangularI = 2047;
    }
    if (vangularI < -2047) {
        vangularI = -2047;
    }

    int vangularAbs = abs(vangularI);
    int vforwardAbs = abs(vforwardI);
    int vnormalAbs  = abs(vnormalI);

    outputBuffer[3] = static_cast<unsigned char>((((vangularAbs >> 7) & 0x0F) << 3) | outputBuffer[3]);

    outputBuffer[4] = static_cast<unsigned char>(vforwardAbs & 0x7F);
    outputBuffer[5] = static_cast<unsigned char>(vnormalAbs & 0x7F);
    outputBuffer[6] = static_cast<unsigned char>((vforwardAbs >> 7) & 0x0F);
    if (vforwardI < 0) {
        outputBuffer[6] = static_cast<unsigned char>(outputBuffer[6] | 0x10);
    }
    if (vnormalI < 0) {
        outputBuffer[6] = static_cast<unsigned char>(outputBuffer[6] | 0x20);
    }

    outputBuffer[7] = static_cast<unsigned char>(vangularAbs & 0x7F);

   /* if (_packet->release) {
        outputBuffer[8] |= 0x01;
    }*/
    outputBuffer[8] |= 0x01;

    if (_packet->chip) {
        outputBuffer[8] = outputBuffer[8] | 0x02;
    }
    if (vangularI < 0) {
        outputBuffer[8] = outputBuffer[8] | 0x04;
    }
    outputBuffer[8] = outputBuffer[8] | (((vnormalAbs >> 7) & 0x0F) << 3);


    outputBuffer[11] = visionCounter & 0x07;
    outputBuffer[12] = 0x3F & _packet->roller_speed;
    outputBuffer[13] = 0;

    for (int k = 0 ; k < _ROBOT_PACKET_SIZE ; k++) {

        robotPackets[robotId][k] = outputBuffer[k];
    }

}

void PacketNodelet::syncData(const parsian_msgs::parsian_world_modelConstPtr &_packet) {

    if (visionCounter < 8) {
        visionCounter ++;
    } else {
        visionCounter = 0;
    }

    robotPacks.value.clear();
    parsian_msgs::parsian_robot_packet temp;
    bool validPack = false;
    for (auto &robotPacket : robotPackets) {
        temp.packets.clear();
        validPack = false;
        for (int i = 0 ; i < _ROBOT_PACKET_SIZE ; i ++) {
            if (robotPacket[i] != 255) {
                temp.packets.push_back(robotPacket[i]);
                validPack = true;
            }
        }
        if (validPack) {
            robotPacks.value.push_back(temp);
        }
    }
    robotPacks.header.stamp = ros::Time::now();
    packetPub.publish(robotPacks);
}

void PacketNodelet::timerCb(const ros::TimerEvent &event) {
    if (drawer != nullptr) {
        drawPub.publish(drawer->getDraws());
        drawer->clear();
    }
}
