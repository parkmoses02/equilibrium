// node build-viewer.js [inline fragment output path]
const fs=require('node:fs'),path=require('node:path');
const read=name=>fs.readFileSync(path.join(__dirname,name),'utf8');
const fragment=read('viewer.fragment.html').replace('/*MODEL*/',()=>read('model.js'));
fs.writeFileSync(path.join(__dirname,'index.html'),'<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>DD → DU 궤적 탐색</title>'+fragment+'</html>');
if(process.argv[2])fs.writeFileSync(process.argv[2],fragment);
