import { onLoad, changeModelProperties } from "./aruco";
import { get } from "jquery"

const host = "192.168.0.101:8080/msgs/debug";
onLoad()

setInterval(function () {
    get(host, function (data) {
        changeModelProperties(JSON.parse(data));
    })
}, 100);