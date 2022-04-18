import { ControlCamera, ThreeDRotation } from '@mui/icons-material'
import { useEffect, useRef } from 'react'
import {OrbitControls} from '../three_tools/OrbitControls'
import * as THREE from 'three'
let THREEs = require("three")
// new OrbitControls(THREEs)




let camera;
let scene;
let controls;

let renderer;
function Point_show(props) {

    const three_ref = useRef(null)
    useEffect(() => {
        
        const width = window.innerWidth/2;
        const height = window.innerHeight/2;
        let ans = props.points;


        scene = new THREE.Scene()

        camera = new THREE.PerspectiveCamera(45, width / height, 1, 10000);
        camera.position.set(10, 10, 10);
        camera.lookAt(scene.position);
        camera.updateMatrix();
        const pointSize = 0.01;
        renderer = new THREE.WebGLRenderer();
        renderer.setSize(width, height);
        three_ref.current.appendChild(renderer.domElement);
        controls = new OrbitControls(camera, renderer.domElement);

        let color = new THREE.Color(1, 0, 0)

        let num = ans.length;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(num / 2);
        const colors = new Float32Array(num / 2);
        let j = 0;
        let k = 0;
        for (let i = 0; i < ans.length; i++) {
            if (!isNaN(ans[i])) {
                if (i % 6 < 3) {
                    positions[j++] = ans[i];
                } else {
                    if (ans[i] > 255 || ans[i] < 0) {
                        colors[k++] = Math.random()
                    } else {
                        colors[k++] = ans[i] / 255
                    }
                }
            }
        }
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        geometry.computeBoundingBox();
        const material = new THREE.PointsMaterial({ size: pointSize, vertexColors: true })
        let buff = new THREE.Points(geometry, material);
        scene.add(buff);
        requestAnimationFrame(animate)
        return ()=>{
            three_ref.current.removeChild(three_ref.current.firstChild)
        }
    })
    var animate = () => {
        requestAnimationFrame(animate);
        controls.update()
        renderer.render(scene, camera);
    }
    return (
        <div>


            <div ref={three_ref}>

            </div>
        </div>
    )

}
export default Point_show;