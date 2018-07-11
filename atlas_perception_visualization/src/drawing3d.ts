import {
	Scene,
	PerspectiveCamera,
	WebGLRenderer,
	BoxGeometry,
	MeshBasicMaterial,
	Mesh,
	Color,
	PlaneGeometry,
	TextureLoader,
	DoubleSide,
	Texture
} from 'three';

export default class Drawing3d {
	private scene: Scene;
	private camera: PerspectiveCamera;
    private renderer: WebGLRenderer;
    private cubeObject = this.createCubeObject();
    private planeObject = this.createTexturedPlane();

	constructor(canvas: HTMLCanvasElement) {
		this.scene = new Scene();
		this.camera = new PerspectiveCamera(75, canvas.width / canvas.height, 0.1, 5000);

		this.renderer = new WebGLRenderer({ canvas });

		this.renderer.setClearColor(new Color(1, 1, 1));

		this.renderer.setSize(canvas.width, canvas.height);

		this.scene.add(this.cubeObject);
		this.scene.add(this.planeObject);

		this.camera.position.set(2, 2, 2);
		this.camera.lookAt(0, 0, 0);

		this.animate();
	}

	loadTexture(): Texture {
		return new TextureLoader().load('textures/texture.png');
	}

	createTexturedPlane(): Mesh {
		const planeGeometry = new PlaneGeometry(2, 2, 10, 10);
		planeGeometry.rotateX(Math.PI / 2);
		const planeMaterial = new MeshBasicMaterial({ map: this.loadTexture(), side: DoubleSide });
		return new Mesh(planeGeometry, planeMaterial);
    }
    
    createCubeObject(): Mesh {
        const boxGeometry = new BoxGeometry(1, 1, 1);
		boxGeometry.translate(0, 1, 0);
		const boxMaterial = new MeshBasicMaterial({ color: 0x000000, wireframe: true });
		return new Mesh(boxGeometry, boxMaterial);
    }

	animate() {
		requestAnimationFrame(this.animate.bind(this));
		this.renderer.render(this.scene, this.camera);
	}
}
