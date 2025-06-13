// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);


//Declare some helper types and functions for calculating CRC32
typedef struct{
	uint32_t polynomial; //The polynomial to use in the CRC calculation
	uint32_t initval; //The "reset" value of the CRC calculation
	bool input_reflect; //Chooses whether bit-reversal is performed on bytes that are fed into the calculation
	bool output_reflect; //Chooses whether bit-reversal is performed on the result of the calculation before returning it
	uint32_t output_xor; //Chooses the value to xor result of the calculation with when returning it
}crc32_parameters_t;
uint32_t compute_CRC32( ApplicationAsset& asset, uint32_t pad_sz, crc32_parameters_t *params );
uint32_t compute_CRC32(const void *data, uint32_t data_sz, uint32_t pad_sz, bool reset, crc32_parameters_t *params);

static uint8_t pad_byte = 0xFF;
crc32_parameters_t crc32_mpeg2 = {
    .polynomial = 0x04C11DB7,
    .initval = 0xFFFFFFFF,
    .input_reflect = false,
    .output_reflect = false,
    .output_xor = 0x00000000
};


//Declare funciton for benchmarking the asset readback process
void asset_readback_benchmark(unsigned int linenum);


//Declare logging function to output log info to serial monitor
SerialLogHandler logHandler(LOG_LEVEL_INFO);




//This function's benchmarks are fast at first (sub 300 ms) but jump to more than 16000 ms towards end of setup function
void setup1() {
    // Put initialization like pinMode and begin functions here
    Serial.begin(230400);
    waitFor(Serial.isConnected, 1000);
    asset_readback_benchmark(__LINE__); //! 245 ms
    //Proxy for linknet_init();
    Serial1.begin(230400);
    asset_readback_benchmark(__LINE__); //! 239 ms
    //Calculate the CRC32 of the Asset
    auto assets = System.assetsAvailable();
    for (auto& asset: assets) {
        if (asset.name() == "BossApp.bin") {
            uint32_t crcResult;
            crcResult = compute_CRC32(
                asset, (512*1024) - asset.size(), &crc32_mpeg2
            );
            Log.info("CRC32 of %s is 0x%08X", asset.name().c_str(), crcResult);
        }
    }
    asset_readback_benchmark(__LINE__); //! 16595 ms
}


//This function's benchmarks are fast throughout the function after adding enclosing scope brackets around the auto assets block
void setup2() {
    // Put initialization like pinMode and begin functions here
    Serial.begin(230400);
    waitFor(Serial.isConnected, 1000);
    asset_readback_benchmark(__LINE__); //! 233 ms
    //Proxy for linknet_init();
    Serial1.begin(230400);
    asset_readback_benchmark(__LINE__); //! 234 ms
    //Calculate the CRC32 of the Asset
    { //! <--- The only difference between setup1() and setup2() is that this block is wrapped in scope brackets
        auto assets = System.assetsAvailable();
        for (auto& asset: assets) {
            if (asset.name() == "BossApp.bin") {
                uint32_t crcResult = compute_CRC32(
                    asset, (512*1024) - asset.size(), &crc32_mpeg2
                );
                Log.info("CRC32 of %s is 0x%08lX", asset.name().c_str(), crcResult);
            }
        }
    } //! <--- The only difference between setup1() and setup2() is that this block is wrapped in scope brackets
    asset_readback_benchmark(__LINE__); //! 270 ms (still fast)
}


//This function's 
void setup3() {
    // Put initialization like pinMode and begin functions here
    Serial.begin(230400);
    waitFor(Serial.isConnected, 1000);
    asset_readback_benchmark(__LINE__); //! 233 ms
    //Proxy for linknet_init();
    Serial1.begin(230400);
    asset_readback_benchmark(__LINE__); //! 234 ms
    //Calculate the CRC32 of the Asset
    auto assets = System.assetsAvailable();
    for (auto& asset: assets) {
        if (asset.name() == "BossApp.bin") {
            uint32_t crcResult = 0x12345678;  //! <---- This time we are not calculating the CRC32, but just setting a dummy value
            Log.info("CRC32 of %s is 0x%08lX", asset.name().c_str(), crcResult);
        }
    }
    asset_readback_benchmark(__LINE__); //! 240 ms (still fast)
}

void setup(){
    // setup1(); //Benchmark slow at end of setup, but fast at beginning, and back to fast again during loop
    // setup2(); //Remains fast throughout setup and loop, because of addition of enclosing scope brackets?
    setup3(); //Remains fast throughout setup and loop, because no CRC32 calculation was performed?
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  if(Serial.available()){
    System.dfu();
  }
  delay(1000);
  asset_readback_benchmark(__LINE__); //! 429 ms
}


unsigned int asset_readback_test(ApplicationAsset& asset)
{
    //Print out information about the asset
    asset.reset();
    size_t assetSize = asset.size();
    Log.info("------------------------------------");
    Log.info("Asset name: %s", asset.name().c_str());
    Log.info("Asset size: %u bytes", assetSize);
    //Now loop throug the asset, reading 1024 bytes at a time
    char buffer[1024];
    size_t bytesRead = 0;
    unsigned int loops = 0;
    unsigned int startTime = millis();
    while (bytesRead < assetSize) {
        Watchdog.refresh(); // Reset the watchdog timer
        size_t bytesToRead = min(assetSize - bytesRead, sizeof(buffer));
        int readResult = asset.read(buffer, bytesToRead);
        if (readResult != (int)bytesToRead) {
            Log.error("Failed to read %u bytes from asset, got %d error code", bytesToRead, readResult);
            break;
        }
        bytesRead += bytesToRead;
    }
    Log.info("Read %u bytes from asset in %u ms", bytesRead, millis() - startTime);
    Log.info("------------------------------------");
   return millis() - startTime;
}

void asset_readback_benchmark(unsigned int linenum)
{
    //This function simply loops through the OTA assets, and tries to read back the first 10 bytes of each 
    //bin file and print them to the log.
    Log.info("Starting asset readback unit test (linenum %u)", linenum);
    auto assets = System.assetsAvailable();
    for (auto& asset: assets) {
        if (asset.name() == "BossApp.bin") {
            asset_readback_test(asset);
        }
    }
}


uint32_t compute_CRC32(
	ApplicationAsset& asset, uint32_t pad_sz, crc32_parameters_t *params
){

    
    asset.reset();
    Log.info("Calculating asset CRC of %s", asset.name().c_str());
    uint32_t crc_result = params->initval;
    unsigned int loops = 0;
    unsigned int total_asset_bytes = asset.size();
    unsigned int asset_bytes_remaining = total_asset_bytes;
    unsigned int total_bytes = total_asset_bytes + pad_sz;
    unsigned int total_bytes_remaining = total_bytes;
    while (asset_bytes_remaining) {
        uint8_t buf[2048];
        //Determine how many bytes to read from the asset
        size_t bytes_to_read = min(asset_bytes_remaining, sizeof(buf));
        int read = asset.read((char*) buf, bytes_to_read);
        if (read < 0) {
            Log.error("Error %d reading binary from asset", read);
            return 0; // Return 0 on error
        }
        crc_result = compute_CRC32(
            buf, bytes_to_read, 0, loops == 0, params
        );
        asset_bytes_remaining -= bytes_to_read;
        loops++;
        if (loops % 10 == 0) {
            Log.info("CRC32 loop %u, progress: %3.1f %%", loops,  (float(total_asset_bytes - asset_bytes_remaining)*100)/total_asset_bytes );
        }
    }
    if(pad_sz > 0) {
        crc_result = compute_CRC32(
            nullptr, 0, pad_sz, loops == 0, params
        );
    }
    return crc_result;
}

static uint8_t Reflect8(uint8_t val)
{
	uint8_t res = 0;
	for (int i = 0; i < 8; i++) {
		if (val & (1 << i)) {
			res |= (1 << (7 - i));
		}
	}
	return res;
}

static uint32_t Reflect32(uint32_t val)
{
	uint32_t res = 0;
	for (int i = 0; i < 32; i++) {
		if (val & (1 << i)) {
			res |= (1 << (31 - i));
		}
	}
	return res;
}

uint32_t compute_CRC32(
	const void *data, uint32_t data_sz, uint32_t pad_sz, bool reset, crc32_parameters_t *params
)
{
	static uint32_t _crc32 = params->initval;
	uint8_t *read_ptr;
	if(data)
		read_ptr = (uint8_t *)data;

	if (reset)
		_crc32 = params->initval;
	uint8_t Byte;
	uint8_t curByte;
	do {
		if (data_sz) {
			Byte = *(read_ptr++);
			data_sz--;
		}
		else {
			Byte = pad_byte;
			pad_sz--;
		}
		/* reflect input byte if specified, otherwise input byte is taken as it is */
		curByte = (params->input_reflect ? Reflect8(Byte) : Byte);
		_crc32 = _crc32 ^ (curByte << 24);
		for (int bit = 0; bit < 8; bit++) {
			if (_crc32 & (1L << 31))
				_crc32 = (_crc32 << 1) ^ (params->polynomial);
			else
				_crc32 = (_crc32 << 1);
		}
	} while (data_sz || pad_sz);
	/* reflect result _crc32if specified, otherwise calculated _crc32value is taken as it is */
	_crc32 = (params->output_reflect ? Reflect32(_crc32) : _crc32);
    /* Xor the crc value with specified final XOR value before returning */
    return (uint32_t)(_crc32 ^ params->output_xor);
}