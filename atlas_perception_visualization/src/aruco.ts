import { AR, POS1 as POS } from 'js-aruco';
import { webcam as camera_settings } from './camera_settings';
import {
    Scene,
    PerspectiveCamera,
    WebGLRenderer,
    Object3D,
    MeshBasicMaterial,
    Mesh,
    Quaternion,
    Euler,
    Matrix4,
    Vector3,
    BoxGeometry,
} from 'three';

var video, videoCanvas, ThreeJSCanvas, context, imageData, detector, posit;
var renderer;
var scene, camera, model, movingModel;
var modelSize = 0.155;

export function onLoad() {
    video = document.getElementById("video");
    videoCanvas = document.getElementById("canvas") as HTMLCanvasElement;
    context = videoCanvas.getContext("2d");

    videoCanvas.width = parseInt(videoCanvas.style.width);
    videoCanvas.height = parseInt(videoCanvas.style.height);

    ThreeJSCanvas = document.getElementById("canvas2");

    ThreeJSCanvas.width = parseInt(ThreeJSCanvas.style.width);
    ThreeJSCanvas.height = parseInt(ThreeJSCanvas.style.height);

    if (navigator.mediaDevices === undefined) {
        throw new Error("no media devices found!");
    }

    if (navigator.mediaDevices.getUserMedia === undefined) {
        throw new Error("no media devices found!");
    }


    navigator.mediaDevices
        .getUserMedia({ video: true })
        .then(function (stream) {
            console.log(video);
            if ("srcObject" in video) {
                video.srcObject = stream;
            } else {
                video.src = window.URL.createObjectURL(stream);
            }
        })
        .catch(function (err) {
            console.log(err.name + ": " + err.message);
        }
        );

    detector = new AR.Detector();
    let camera_info = camera_settings.camera_matrix.data;
    let fx = camera_info[0];
    let fy = camera_info[4];
    let focalLength = (fx + fy) / 2;
    let fov = 2 * Math.atan2(camera_settings.image_height, (2 * fy)) * 180 / Math.PI;
    posit = new POS.Posit(modelSize, focalLength);
    createRenderers(fov);
    createScenes();

    requestAnimationFrame(tick);
}

export function changeModelProperties(message) {

    let object_message = message.detectedObjects[0]
    /*console.log(message);


    let transformationMat = new Matrix4();
    let quaternion = new Quaternion();

    /*let position = new Vector3();
    position.x = -1;
    position.y = 1;
    position.z = -1;

    let scale = new Vector3();
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;

    let rotation = new Euler();
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    quaternion.setFromEuler(rotation)
    transformationMat.compose(position,quaternion,scale)
*/
    movingModel.scale.x = object_message.boundingBox.size.width;
    movingModel.scale.y = object_message.boundingBox.size.height;
    movingModel.scale.z = object_message.boundingBox.size.depth;

    movingModel.position.x = object_message.boundingBox.transform.translation.x;
    movingModel.position.y = object_message.boundingBox.transform.translation.y;
    movingModel.position.z = object_message.boundingBox.transform.translation.z;

    movingModel.rotation.x = object_message.boundingBox.transform.rotation.x;
    movingModel.rotation.y = object_message.boundingBox.transform.rotation.y;
    movingModel.rotation.z = object_message.boundingBox.transform.rotation.z;
    movingModel.rotation.w = object_message.boundingBox.transform.rotation.w;

    // movingModel.rotation.x = 0;
    // movingModel.rotation.y = 0;
    // movingModel.rotation.z = 0;

    // movingModel.position.x = 0;
    // movingModel.position.y = 0;
    // movingModel.position.z = -1;

    console.log("Object set");
    // console.log(movingModel.position)*/

}

function createScenes() {
    model = createModel();
    movingModel = createMovingObject();
    scene.add(model);
    scene.add(movingModel);
}

function createModel() {
    var object = new Object3D(),
        geometry = new BoxGeometry(1, 1, 1),
        material = new MeshBasicMaterial({ color: 0xFFFFFF, wireframe: true }),
        mesh = new Mesh(geometry, material);
    object.position.x = 0;
    object.position.y = 0;
    object.position.z = modelSize / 2;

    object.add(mesh);

    return object;
}

function createMovingObject() {
    var object = new Object3D(),
        geometry = new BoxGeometry(1, 1, 1),
        material = new MeshBasicMaterial({ color: 0xFF0000, wireframe: true });
    var mesh = new Mesh(geometry, material);

    object.add(mesh);
    object.position.x = 0;
    object.position.y = 1;
    object.position.z = -2;
    // object.position.z = modelSize;

    object.rotation.x = 0;
    object.rotation.y = 0;
    object.rotation.z = 0;

    object.scale.x = modelSize;
    object.scale.y = modelSize;
    object.scale.z = modelSize;

    return object;
}


function createRenderers(fov) {
    renderer = new WebGLRenderer({ canvas: ThreeJSCanvas, alpha: true });
    renderer.setSize(videoCanvas.width, videoCanvas.height);
    scene = new Scene();
    camera = new PerspectiveCamera(fov, videoCanvas.width / videoCanvas.height, 0.1, 1000);
    scene.add(camera);
}

function render() {
    renderer.autoClear = false;
    renderer.clear();
    renderer.render(scene, camera);
}


function estimatePose(markers) {
    var corners, corner, pose, i;

    if (markers.length > 0) {
        corners = markers[0].corners;

        for (let i = 0; i < corners.length; ++i) {
            corner = corners[i];

            corner.x = corner.x - (videoCanvas.width / 2);
            corner.y = (videoCanvas.height / 2) - corner.y;
        }

        pose = posit.pose(corners);

        updateCameraPose(model, pose.bestRotation, pose.bestTranslation);
        model.visible = true;
        movingModel.visible = true;
    }
    else {
        model.visible = false;
        movingModel.visible = false;
    }
}

function updateCameraPose(object, markerRotation, translation) {
    let scale = new Vector3();
    let rotation = new Euler();
    let position = new Vector3();
    let quaternion = new Quaternion();

    // scale.x = modelSize;
    // scale.y = modelSize;
    // scale.z = modelSize;
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;

    model.scale.x = modelSize;
    model.scale.y = modelSize;
    model.scale.z = modelSize;

    let poseMat = new Matrix4();

    poseMat.set(markerRotation[0][0], markerRotation[0][1], markerRotation[0][2], translation[0],
        markerRotation[1][0], markerRotation[1][1], markerRotation[1][2], translation[1],
        markerRotation[2][0], markerRotation[2][1], markerRotation[2][2], translation[2],
        0, 0, 0, 1);

    //translate to ThreeJS reference frame
    position.x = translation[0];
    position.y = translation[1];
    position.z = -translation[2];


    quaternion.setFromRotationMatrix(poseMat);
    rotation.setFromQuaternion(quaternion);


    //rotate to ThreeJS reference frame
    rotation.y = -rotation.y;
    rotation.x = -rotation.x;

    quaternion.setFromEuler(rotation);
    let invMat = new Matrix4();
    poseMat.compose(position, quaternion, scale);

    invMat.getInverse(poseMat);

    // multipy by model matrix to place camera relative to object
    //invMat.multiplyMatrices(model.matrix, invMat);
    invMat.decompose(position, quaternion, scale);

    camera.position.x = position.x;
    camera.position.y = position.y;
    camera.position.z = position.z;
    camera.rotation.setFromQuaternion(quaternion);

};

function tick() {
    if (video.readyState === video.HAVE_ENOUGH_DATA) {
        snapshot();

        var markers = detector.detect(imageData);
        drawCorners(markers);
        drawId(markers);
        estimatePose(markers);

        render();
    }
    requestAnimationFrame(tick);
}

function snapshot() {
    context.drawImage(video, 0, 0, videoCanvas.width, videoCanvas.height);
    imageData = context.getImageData(0, 0, videoCanvas.width, videoCanvas.height);
}

function drawCorners(markers) {
    var corners, corner, i, j;

    context.lineWidth = 3;

    for (i = 0; i !== markers.length; ++i) {
        corners = markers[i].corners;

        context.strokeStyle = "red";
        context.beginPath();

        for (j = 0; j !== corners.length; ++j) {
            corner = corners[j];
            context.moveTo(corner.x, corner.y);
            corner = corners[(j + 1) % corners.length];
            context.lineTo(corner.x, corner.y);
        }

        context.stroke();
        context.closePath();

        context.strokeStyle = "green";
        context.strokeRect(corners[0].x - 2, corners[0].y - 2, 4, 4);
    }
}

function drawId(markers) {
    var corners, corner, x, y, i, j;

    context.strokeStyle = "blue";
    context.lineWidth = 1;

    for (i = 0; i !== markers.length; ++i) {
        corners = markers[i].corners;

        x = Infinity;
        y = Infinity;

        for (j = 0; j !== corners.length; ++j) {
            corner = corners[j];
            x = Math.min(x, corner.x);
            y = Math.min(y, corner.y);
        }

        context.strokeText(markers[i].id, x, y)
    }
}
