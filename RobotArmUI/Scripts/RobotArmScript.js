(function (doc, win, $, myModule) {
    var scene;
    var renderer;
    var camera;
    var points = [];
    var projector;
    var mouseVector;
    var CANVAS_WIDTH;
    var CANVAS_HEIGHT;
    var latestRobotArmComponents = [];
    var highlightedPoint;

    function initializeScene() {
        var scene3d = document.getElementById("robotArmGraphic");

        var widthR = $("#robotArmGraphic").width();
        var heightR = $("#robotArmGraphic").height();
        CANVAS_WIDTH = widthR;
        CANVAS_HEIGHT = heightR;
        //points = new THREE.Object3D();
        projector = new THREE.Projector();
        mouseVector = new THREE.Vector3();

// SCENE
        scene = new THREE.Scene();
        //scene.position.x = 10;

// CAMERA 
        initCamera(CANVAS_WIDTH, CANVAS_HEIGHT);

// RENDERER
        renderer = new THREE.WebGLRenderer();
        renderer.setClearColor(0x000, 1.0);
        renderer.setSize(CANVAS_WIDTH, CANVAS_HEIGHT);

// GEOMETRY & MATERIALS

//        var cubeGeometry = new THREE.BoxGeometry(3, 3, 3);
//        var cubeMaterial = new THREE.MeshLambertMaterial({ color: 0xff55ff });
//        var cube = new THREE.Mesh(cubeGeometry, cubeMaterial);
////        scene.add(cube);
//        cube.position.z = 4;

        

//        var floorGeometry = new THREE.BoxGeometry(30, 1, 30);
//        var floorMaterial = new THREE.MeshBasicMaterial({ color: 0x656587 });
//        var floor = new THREE.Mesh(floorGeometry, floorMaterial);
////        scene.add(floor);
//        floor.position.y = -3;
//        floor.receiveShadow = true;

        drawAxes();
//        line.position.z = 5;

// LIGHT
        initLight();
// FINISH SCENE SETUP

// document.body.appendChild(scene3d.domElement);
        scene3d.appendChild(renderer.domElement);
        renderer.render(scene, camera);
       // window.addEventListener('mousemove', onMouseMove, false);
    }

    function initCamera(CANVAS_WIDTH, CANVAS_HEIGHT) {
        camera = new THREE.PerspectiveCamera(45, CANVAS_WIDTH / CANVAS_HEIGHT, 0.1, 1000);
        camera.position.x = 5;
        camera.position.y = 0;
        camera.position.z = 40;
        //camera.lookAt(0,0,-40);
    }

    function initLight() {
        var spot1 = new THREE.SpotLight(0xffffff);
        spot1.position.set(0, 0, 20);
        scene.add(spot1);
    }

    function drawAxes() {
        var axisX = new THREE.Geometry();
        axisX.vertices.push(new THREE.Vector3(-100, 0, 0));
        axisX.vertices.push(new THREE.Vector3(100, 0, 0));
        var material = new THREE.LineBasicMaterial({ color: 0xFF0000, linewidth: 1 });
        var line = new THREE.Line(axisX, material);
        scene.add(line);
        var axisY = new THREE.Geometry();
        axisY.vertices.push(new THREE.Vector3(0, -100, 0));
        axisY.vertices.push(new THREE.Vector3(0, 100, 0));
        var material = new THREE.LineBasicMaterial({ color: 0xFF0000, linewidth: 1 });
        var line1 = new THREE.Line(axisY, material);
        scene.add(line1);

        drawAxisDashes();

        //var axesHelper = new THREE.AxesHelper(500);
        //scene.add(axesHelper);
    }

    function drawAxisDashes() {
        var material = new THREE.LineBasicMaterial({ color: 0xFF0000, linewidth: 1 });
        for (var i = -100; i <= 100; i++) {
            if (i === 0) continue;
            var dashHeight = i % 5 === 0 ? 0.25 : 0.10;

            if (i % 5 === 0) drawAxisNumber(i);
            var dashX = new THREE.Geometry();
            dashX.vertices.push(new THREE.Vector3(i, dashHeight, 0));
            dashX.vertices.push(new THREE.Vector3(i, -dashHeight, 0));
            var lineX = new THREE.Line(dashX, material);
            scene.add(lineX);
            var dashX = new THREE.Geometry();
            dashX.vertices.push(new THREE.Vector3(dashHeight, i, 0));
            dashX.vertices.push(new THREE.Vector3(-dashHeight, i, 0));
            var lineX = new THREE.Line(dashX, material);
            scene.add(lineX);
        }
    }

    function drawAxisNumber(number) {
        var x = document.createElement("canvas");
        var xc = x.getContext("2d");
        x.width = x.height = 64;
        //xc.shadowColor = "#000";
        xc.shadowBlur = 0;
        xc.fillStyle = "white";
        xc.font = "30pt arial bold";
        xc.fillText(number, 0, 64);

        var xm = new THREE.MeshBasicMaterial({ map: new THREE.Texture(x), transparent: true });
        xm.map.needsUpdate = true;

        var meshX = new THREE.Mesh(new THREE.CubeGeometry(1, 1, 0), xm);
        meshX.position.x = number + 0.20;
        meshX.position.y =  -0.5;
        meshX.position.z = 0;
        meshX.doubleSided = false;
        scene.add(meshX);
        var meshY = new THREE.Mesh(new THREE.CubeGeometry(1, 1, 0), xm);
        meshY.position.x = -0.5;
        meshY.position.y =  number + 0.1;
        meshY.position.z = 0;
        meshY.doubleSided = false;
        scene.add(meshY);
    }

    function drawPoint(x, y, z, options) {
        options = options !== undefined ? options : {};
        const ballGeometry = new THREE.SphereGeometry(0.1, 16, 16);
        const ballMaterial = new THREE.MeshPhongMaterial({ color: options.type !== undefined ? (options.type === "robotArmComponent" || options.type === "highlightPosition") ? 0xFF0000 : options.type === "experiment" ? 0xFFFB00 : 0x33aaff  : 0x33aaff });
        const ball = new THREE.Mesh(ballGeometry, ballMaterial);
        scene.add(ball);
        ball.position.z = z;
        ball.position.x = x;
        ball.position.y = y;
        points.push(ball);
        if (options.type !== undefined && options.type === "robotArmComponent") {
            ball.name = "robotArmComponent";
            latestRobotArmComponents.push(ball);
        }
        else if (options.type !== undefined && options.type === "highlightPosition") {
            ball.name = "highlightPosition";
            highlightedPoint = ball;
        }
    }

    function drawRobotArm(jointPoint, endPoint) {
        const material = new THREE.LineBasicMaterial({ color: randomColor() });
        const robotArm = new THREE.Geometry();
        robotArm.vertices.push(new THREE.Vector3( 0, 0, 0) );
        robotArm.vertices.push(new THREE.Vector3( jointPoint.x, jointPoint.y, jointPoint.z) );
        robotArm.vertices.push(new THREE.Vector3( endPoint.x, endPoint.y, endPoint.z) );  
        const line = new THREE.Line(robotArm, material);
        line.name = `robotArmComponent`;
        scene.add(line);
        latestRobotArmComponents.push(line);
        drawPoint(jointPoint.x, jointPoint.y, jointPoint.z, {type: "robotArmComponent"});
        drawPoint(endPoint.x, endPoint.y, endPoint.z, { type: "robotArmComponent" });
    }

    function randomColor() {
        const colorNum = Math.floor(Math.random() * (5 - 0 + 1)) + 0;
        switch(colorNum) {
            case 0: return 0x0000FF;
            case 1: return 0x3ADF00;
            case 2: return 0x82FA58;
            case 3: return 0x2EFEF7;
            case 4: return 0xFE2EF7;
            case 5: return 0xFAAC58;
        }
    }

    function renderScene() {
        renderer.render(scene, camera);
    }

    function changeCameraPosition(side) {
        switch (side) {
            case "left":
                camera.position.x -= 5;
                break;
            case "right":
                camera.position.x += 5;
                break;
            case "up":
                camera.position.y += 5;
                break;
            case "down":
                camera.position.y -= 5;
                break;
            case "in":
                camera.position.z -= 5;
                break;
            case "out":
                camera.position.z += 5;
                break;
        default:
        }
//        controls.update();
    }

    //function onMouseMove(e) {
    //    if (points.length === 0) return;
    //    mouseVector.x = 2 * (e.clientX / window.innerWidth) - 1;
    //    mouseVector.y = 1 - 2 * (e.clientY / window.innerHeight);
    //    //console.log(mouseVector);
    //    var raycaster = new THREE.Raycaster();
    //    raycaster.setFromCamera(mouseVector.clone(), camera);
    //     var intersects = raycaster.intersectObjects(points);

    //    points.forEach(function (point) {
    //        //point.material.color.setRGB(51, 170, 255);
    //    });


    //    for (var i = 0; i < intersects.length; i++) {
    //        var intersection = intersects[i];
    //        console.log(intersects[i]);
    //          var  obj = intersection.object;

    //        obj.material.color.setRGB(1.0 - i / intersects.length, 0, 0);
    //    }

    //    renderScene();

    //}

    function clearScene() {
        while (scene.children.length > 0) {
            scene.remove(scene.children[0]);
        }
        drawAxes();
        initLight();
        renderScene();
    }

    function clearLatestRobotArmComponents() {
        if (latestRobotArmComponents.length === 0) return;
        latestRobotArmComponents.map(object => {
            const selectedObject = scene.getObjectByName(object.name);
            scene.remove(selectedObject);
        });
        latestRobotArmComponents = [];
        renderScene();
    }

    function highlightPosition(x, y, z) {
        if (highlightedPoint !== undefined) {
            const selectedObject = scene.getObjectByName(highlightedPoint.name);
            scene.remove(selectedObject);
        }
        drawPoint(x, y, z, { type: "highlightPosition" });
        renderScene();
    }

    
    window.RobotArm = {
        InitializeScene: initializeScene,
        DrawPoint: drawPoint,
        RenderScene: renderScene,
        ChangeCameraPosition: changeCameraPosition,
        ClearScene: clearScene,
        DrawRobotArm: drawRobotArm,
        ClearLatestRobotArmComponents: clearLatestRobotArmComponents,
        HighlightPosition: highlightPosition
    }
}(document, window, jQuery, window.RobotArm));