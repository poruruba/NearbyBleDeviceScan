'use strict';

const HELPER_BASE = process.env.HELPER_BASE || "/opt/";
const Response = require(HELPER_BASE + 'response');

let device_list = new Map();

exports.mqtt_handler = async (event, context) => {
  console.log(event);

  for( let device of event.payload.device_list ){
    device.host_address = event.payload.ble_mac_address;
    device_list.set(device.mac_address, device);
  }
};

exports.handler = async (event, context, callback) => {
	var body = JSON.parse(event.body);
	console.log(body);
  if( event.path == "/bledevice-list" ){
  	return new Response({ list: [...device_list.values()] });
  }else if( event.path == "/bledevice-put" ){
    for( let device of body.device_list ){
      device.host_address = body.ble_mac_address;
      device_list.set(device.mac_address, device);
    }
    return new Response({});
  }
};
