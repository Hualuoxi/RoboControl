#include "gyro.h"

Gyro::Gyro()
{
    mUSART = new USART(usart1);
    mUSART->startRcv();
    connect(mUSART,&USART::sendGyroDate_signal,this,&Gyro::setAngleNow_slot);
}

unsigned char Gyro::check_data_len_by_id(unsigned char id, unsigned char len, unsigned char *data)
{
    unsigned char ret = 0xff;
    switch(id)
    {
        case ACCEL_ID:
        {
            if(ACCEL_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.accel_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
                g_output_info.accel_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                g_output_info.accel_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;
        case ANGLE_ID:
        {
            if(ANGLE_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.angle_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
                g_output_info.angle_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                g_output_info.angle_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;

        case MAGNETIC_ID:
        {
            if(MAGNETIC_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.mag_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
                g_output_info.mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                g_output_info.mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;

        case RAW_MAGNETIC_ID:
        {
            if(MAGNETIC_RAW_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.raw_mag_x = get_signed_int(data) * MAG_RAW_DATA_FACTOR;
                g_output_info.raw_mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
                g_output_info.raw_mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;

        case EULER_ID:
        {
            if(EULER_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
                g_output_info.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                g_output_info.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;

        case QUATERNION_ID:
        {
            if(QUATERNION_DATA_LEN == len)
            {
                ret = (unsigned char)0x1;
                g_output_info.quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
                g_output_info.quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                g_output_info.quaternion_data2 = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                g_output_info.quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = (unsigned char)0x00;
            }
        }
        break;

        default:
        break;
    }

    return ret;
}

/*--------------------------------------------------------------------------------------------------------------
* 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* crc校验从TID开始到payload data的最后一个字节
*/
int Gyro::analysis_data(unsigned char *data, short len)
{
    unsigned short payload_len = 0;
    unsigned short check_sum = 0;
    unsigned short pos = 0;
    unsigned char ret = 0xff;

    output_data_header_t *header = NULL;
    payload_data_t *payload = NULL;

    if(NULL == data || 0 >= len)
    {
        return para_err;
    }

    if(len < PROTOCOL_MIN_LEN)
    {
            return data_len_err;
    }

    /*judge protocol header*/
    if(PROTOCOL_FIRST_BYTE == data[PROTOCOL_FIRST_BYTE_POS] && \
        PROTOCOL_SECOND_BYTE == data[PROTOCOL_SECOND_BYTE_POS])
    {
        /*further check*/
        header = (output_data_header_t *)data;
        payload_len = header->len;

        if(payload_len + PROTOCOL_MIN_LEN > len)
        {
            return 	data_len_err;
        }

        /*checksum*/
        calc_checksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
        if(check_sum != ((unsigned short)(data[PROTOCOL_CRC_DATA_POS(payload_len)])<<8|data[PROTOCOL_CRC_DATA_POS(payload_len)+1]))
        {
            return crc_err;
        }

        /*analysis payload data*/
        pos = PAYLOAD_POS;

        while(payload_len > 0)
        {
            payload = (payload_data_t *)(data + pos);
            ret = check_data_len_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);
            if((unsigned char)0x01 == ret)
            {
                pos += payload->data_len + sizeof(payload_data_t);
                payload_len -= payload->data_len + sizeof(payload_data_t);
            }
            else
            {
                pos++;
                payload_len--;
            }
        }

        return analysis_ok;
    }
    else
    {
        return analysis_done;
    }
}

int Gyro::get_signed_int(unsigned char *data)
{
    int temp = 0;

    temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

    return temp;
}

int Gyro::calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
    unsigned char check_a = 0;
    unsigned char check_b = 0;
    unsigned short i;

    if(NULL == data || 0 == len || NULL == checksum)
    {
        return -1;
    }

    for(i = 0; i < len; i++)
    {
//        qDebug()<<hex<<data[i]<<endl;
        check_a += data[i];
        check_b += check_a;
    }
//    qDebug()<<"check_a:"<<hex<<check_a<<" check_b:"<<hex<<check_b;
    *checksum = ((unsigned short)(check_a << 8) | check_b);

    return 0;
}

RobotAngle Gyro::getAngleNow()
{
    QMutexLocker AngleNow_MutexLocker(&AngleNow_Mutex);
    return AngleNow;
}

void Gyro::setAngleNow_slot(QVector<u8>a)
{
//    qDebug()<<"setAngleNow_slot"<<QThread::currentThread();
//    for(int i=0;i<a.length();i++)
//        qDebug()<<hex<<a[i]<<endl;
    res = analysis_data(&a[0], a.length());
//    qDebug()<<"the time1 is : "<<begin_time.msecsTo(QDateTime::currentDateTime());
    if(res == 0 || res == 1)
    {
        QMutexLocker AngleNow_MutexLocker(&AngleNow_Mutex);
        AngleNow.yaw = g_output_info.yaw-AngleRef.yaw;
        AngleNow.roll = g_output_info.roll-AngleRef.roll;
        AngleNow.pitch = g_output_info.pitch-AngleRef.pitch;
//        qDebug()<<QString("yaw = %1 ,roll = %2 ,pitch = %3,AngleRef.yaw = %4").arg(AngleNow.yaw).arg(AngleNow.roll).arg(AngleNow.pitch).arg(AngleRef.yaw);
        if(SvRefAng)
        {
            AngleRef=AngleNow;
            SvRefAng = false;
        }
//        qDebug()<<"Yaw: "<<AngleNow.yaw<<"roll: "<<AngleNow.roll<<"pitch: "<<AngleNow.pitch;
    }
//    else
//    {
//        qDebug()<<res<<"analysis_data error";
//    }
}
