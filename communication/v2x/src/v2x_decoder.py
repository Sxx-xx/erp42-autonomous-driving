'''
© 2022 Hansung Kim <hansung@berkeley.edu>
SPAT Decoder per SAE-J2735-2020 standard
'''
import asn1tools
import os

'''
Before using this decoder, complete the following steps:
1) Download the .zip file from https://www.sae.org/standards/content/j2735asn_202007/
2) Extract the contents into a folder
'''

class J2735Decoder:
    def __init__(self):
        self.spat = self.compile_asn1()
        print("SPAT decoder instance created")

    def compile_asn1(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        asn1_path = os.path.join(script_dir, "..", "SAE-J2735-2020")

        # List of .asn filenames to compile
        filenames = [os.path.join(asn1_path, filename) for filename in os.listdir(asn1_path) if filename.endswith('.asn')]

        # Compile (Compiling takes long, so it should only be done once)
        spat = asn1tools.compile_files(filenames, 'uper')
        return spat

    def spat_decoder(self, data):
        decoded = self.spat.decode('SPAT', self.spat.decode('MessageFrame', data)['value'])
        decoded = self.recursive_values_decode_bytes(decoded)
        return decoded
    
    def map_decoder(self, data):
        decoded = self.spat.decode('MapData', self.spat.decode('MessageFrame', data)['value'])
        decoded = self.recursive_values_decode_bytes(decoded)
        return decoded
    
    def decode_hex_bytes(self, hex_bytes):
        hex_string = ''.join(format(byte, '02x') for byte in hex_bytes)
        byte_values = bytes.fromhex(hex_string)
        return int.from_bytes(byte_values, byteorder="big")
    
    def recursive_values_decode_bytes(self, data):
        if isinstance(data, dict):
            return {key: self.recursive_values_decode_bytes(value) for key, value in data.items()}
        elif isinstance(data, list):
            return [self.recursive_values_decode_bytes(item) for item in data]
        elif isinstance(data, tuple) and len(data) == 2 and isinstance(data[0], bytes) and isinstance(data[1], int):
            decoded_value = self.decode_hex_bytes(data[0])
            return (decoded_value, data[1])  # 두 번째 요소는 그대로 둠
        else:
            return data

if __name__ == '__main__':
    import json
    def pretty_print_dict(dictionary, indent=2):
        print(json.dumps(dictionary, indent=indent))
    
    # Decode example SPAT data
    decoder_instance = J2735Decoder()
    # FMTC Data
    # hex_data = "0013808f001880cd00cd01040052bd49b800761e9d4a5064c791500410d0041006a401001839916354020474000a000f0040061e9d4a5064c791500c10d00028076c01001839916354040434000a01db0040061e9d4a5064c791501410d0041006a401001839916354060474000a000f0040061e9d4a5064c791501c10d0021c076c01001839916354080434008701db004000"
    # KCity Data
    hex_data = "001380ad22db3260c22d486c18b064c000200c800c8041c143d02a75290082000000103991635402040000002054ea520304000000207322c6a8080800000040a9d4a40a0800000040e6458d5018100000008153a9481c1000000081cc8b1aa0402000000102a7529048200000010399163540a040000002054ea520b04600064207322c6a8180800000040a9d4a41a0800000040e6458d503810c004ec8153a9483c1180032081cc8b1aa0802180039809209"
    
    if hex_data[:4] == "0013":  # If data is SPAT
        bytes_data = bytes.fromhex(hex_data)
        decoded_spat = decoder_instance.spat_decoder(bytes_data)
        decoded_spat = decoder_instance.recursive_values_decode_bytes(decoded_spat)
        # print(decoded_spat)
        pretty_print_dict(decoded_spat)