import re
import struct
import datetime
import os
import sys

# This script converts the date captured from busmaster to
# the same structure as logged by the sd card on the car
# so the same script can be used t upload to SQL (see at )
#
#  INPUT FORMAT: 
# ***<Time><Tx/Rx><Channel><CAN ID><Type><DLC><DataBytes>***
# Example: 08:47:17:4802 Rx 1 0x08F s 8 00 00 00 00 00 00 00 00
# 
# OUTPUT FORMAT:
# like this:
# void formatMessage(uint32_t id, byte_t* bytes, uint32_t time, uint16_t date, char* message){
#     //requires message to have enough allocated memory
#     union int32_bytes id_bytes;
#     id_bytes.i = id;

#     union int32_bytes time_bytes;
#     time_bytes.i = time;

#     union int16_bytes date_bytes;
#     date_bytes.i = date;

#     for(int i = 0; i < 8; i++) {
#         message[i] = (char) bytes[i];
#     }
#     for(int i = 0; i < 4; i++){
#         message[i + 8] = id_bytes.bytes[i];          //-i to write in the expected byte order
#     }
#     for(int i = 0; i< 4; i++){
#         message[i + 12] = time_bytes.bytes[i];       //-i to write in the expected byte order
#     }
#     for(int i= 0; i< 2;i++){
#         message[i + 16] = date_bytes.bytes[i];       //-i to write in the expected byte order
#     }
# }

def parse_time_to_ms(timestr):
    # timestr: "08:47:17:4802"
    #try:
    t, ms = timestr.rsplit(':', 1)
    dt = datetime.datetime.strptime(t, "%H:%M:%S")
    ms = int(ms[:3])
    total_ms = (dt.hour * 3600 + dt.minute * 60 + dt.second) * 1000 + ms
    return total_ms
    # except Exception:
    #     return 0

def convert_log_format(input_file, output_file):
    """
    Reads CAN messages from input_file in the format:
    <Time> <Tx/Rx> <Channel> <CAN ID> <Type> <DLC> <DataBytes>
    and writes them to output_file in the binary format matching the C function.
    """
    fso = 0
    with open(input_file, 'r') as infile, open(output_file, 'wb') as outfile:
        for line in infile:
            line = line.strip()
            if not line or line.startswith('*'):
                continue
            fields = line.split()
            if len(fields) < 7:
                continue  # Skip malformed lines

            if fields[5] != "8":
                continue

            
            # Extract fields
            time_str = fields[0]
            time = parse_time_to_ms(time_str)
            # fields[1]: Tx/Rx, fields[2]: Channel (not used)
            can_id = int(fields[3], 16)
            if can_id == 1:
                continue
            
            if can_id > 200:
                continue
            
            if fields[4] != "s":
                continue

            dlc = int(fields[5])
            data_bytes = [int(b, 16) for b in fields[6:6+dlc]]

            # Pad data_bytes to 8 bytes
            data_bytes += [0] * (8 - len(data_bytes))

            # For this example, set date to 0 (or extract if available)
            date = 0

            # Pack into binary format (little-endian)
            packed = struct.pack('<8B I I H', *data_bytes, can_id, time, date)
            line = line + "\n"
            #outfile.write(line.encode())
            outfile.write(packed)



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Drag and drop a log file onto this script to convert it.")
        sys.exit(1)
    input_path = sys.argv[1]
    base, _ = os.path.splitext(os.path.basename(input_path))
    # output_path = os.path.join(os.path.dirname(__file__), base + ".bin")
    output_path = os.path.splitext(input_path)[0] + ".canSdLog"

    print(f"Trying to convert file: {input_path}")
    
    convert_log_format(input_path, output_path)
    print(f"Converted '{input_path}' to '{output_path}'")