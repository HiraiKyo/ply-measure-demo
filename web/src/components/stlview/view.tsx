import React, { useEffect, useRef } from "react";
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { styled } from '@mui/material/styles';

interface StlViewerProps {
  url: string;
  width: number;
  height: number;
}

const ViewerContainer = styled('div')({
  width: '100%',
  height: '100%',
});

export const STLViewerComponent: React.FC<StlViewerProps> = ({
  url,
  width,
  height
}) => {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // シーンの設定
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x121212);

    // カメラの設定
    const camera = new THREE.PerspectiveCamera(
      50,
      width / height,
      0.1,
      2000
    );
    camera.position.z = 300;

    // レンダラーの設定
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    containerRef.current.appendChild(renderer.domElement);

    // ライティングの設定
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(0, 0, 2);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x404040));

    // OrbitControlsの設定
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    // STLファイルのロード
    const loader = new STLLoader();
    loader.load(
      url,
      (geometry) => {
        const material = new THREE.MeshPhongMaterial({
          color: 0xffffff,
          specular: 0x111111,
          shininess: 200,
        });
        const mesh = new THREE.Mesh(geometry, material);
        scene.add(mesh);

        // モデルの中心を合わせる
        geometry.center();
        
        // モデルのサイズに応じてカメラ位置を調整
        const box = new THREE.Box3().setFromObject(mesh);
        const size = box.getSize(new THREE.Vector3());
        const maxSize = Math.max(size.x, size.y, size.z);
        camera.position.z = maxSize * 2;
      },
      undefined,
      (error) => {
        console.error('STLファイルの読み込みに失敗しました:', error);
      }
    );

    // アニメーションループ
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // クリーンアップ
    return () => {
      if (containerRef.current) {
        containerRef.current.removeChild(renderer.domElement);
      }
    };
  }, [url, width, height]);

  return <ViewerContainer ref={containerRef} />;
};