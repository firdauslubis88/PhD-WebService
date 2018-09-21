var express = require('express');
var app = express();
var cors = require('cors');
var bodyParser = require('body-parser');
var fs = require('fs');

app.use(cors());
app.use(bodyParser.urlencoded({ limit:'50mb', extended: true }));

app.get('/listusers', function(req, res){
	fs.readFile(__dirname + '/users.json', 'utf8', function(err, data){
		console.log(data);
		res.end(data);
	});
});
app.post('/updatePanTilt', function(req, res){
	var execFile = require('child_process').execFile;
	var program = 'd:/repos/PhD-WebService/PTZControl/bin/PTZControl.exe';
	var args = [];
	if(req.body.pan_value != null){
		args.push('pan');
		args.push(req.body.pan_value);
	}
	if(req.body.tilt_value != null){
		args.push('tilt');
		args.push(req.body.tilt_value);
	}
//	var args = [req.body.direction,req.body.direction_value];
//	console.log(args);
//	res.end(args);
	var child = execFile(program, args, function(error, stdout, stderror){
		var number = stdout;
		console.log(number);
		res.end(number);
	});
	
	
});
app.post('/updateVerticesMap', function(req, res){
	var execFile = require('child_process').execFile;
	var program = 'd:/repos/PhD-WebService/UpdateVerticesMap/bin/Automation.exe';
	
	var args = 100;
	var child = execFile(program, [args], function(error, stdout, stderror){
		var number = stdout;
		console.log(number);
		res.end(number);
	});
	
	
});
app.post('/updateVerticesMap_backup', function(req, res){
	var base64Data = req.body.imgBase64.replace(/^data:image\/png;base64,/, "");
	var thetaBase64Data = req.body.thetaImgBase64.replace(/^data:image\/png;base64,/, "");
	fs.writeFile('ptzInput.png', base64Data, 'base64', function(err){
		if(err)
		{
			console.log(err);
			return;
		}
	});
	fs.writeFile('thetaInput.png', thetaBase64Data, 'base64', function(err){
		if(err)
		{
			console.log(err);
			return;
		}
	});
});

var server = app.listen(8080, function(){
	var host = server.address().address;
	var port = server.address().port;
	console.log('WEB SERVICE is listening at http://%s:%s', host, port);
});