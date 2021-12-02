<html>
<head>
<title>Fourtor Data logging</title>
<!-- JQuery & Highchart 스크립트 추가 -->
<script src="http://ajax.googleapis.com/ajax/libs/jquery/1.8.2/jquery.min.js"></script>
<script src="http://code.highcharts.com/highcharts.js"></script>
<script>
<?php
$dbhost = '퍼블릭 주소';
$dbuser = '사용자';
$dbpass = '비밀번호';
$dbname = 'DB이름'; //Create database connection

$dblink = new mysqli($dbhost, $dbuser, $dbpass, $dbname); //Check connection was successful
if ($dblink->connect_errno) { printf("Failed to connect to database"); exit(); } //Fetch 3 rows from actor table
$result = $dblink->query("SELECT * FROM test"); //Initialize array variable
$dbdata = array(); //Fetch into associative array
$rpm = array();
$Battery_temp=array();
$Battery_vol=array();
$Motor_temp=array();
$Motor_torq=array();
while ( $row = $result->fetch_assoc()) {
	$dbdata[]=1000*strtotime($row['timestamp']);
	$Battery_temp[]=$row['temp_max'];
	$Battery_vol[]=$row['B_vol_int']+0.1*$row['B_vol_div'];
	$Motor_temp[]=$row['motor_temp'];
	$Motor_torq[]=$row['motor_torq'];
	$rpm[]=$row['rpm'];
} //Print array in JSON format
?>
$(function () {
    $('#container').highcharts({
        chart: {
	    type: 'spline',
	    backgroundColor:null,
		events:{
		load:
		function(){
			var series = this.series[0];
			var series2 = this.series[1];
			var series3 = this.series[2];
			var series4 = this.series[3];
			var series5 = this.series[4];
			var categories = this.xAxis[0].categories.slice(0);
			chart=this;
			setInterval(function() {
			 $.ajax({
   			 url: './good.php',
   			 type: 'GET',
   			 async: true,
    			dataType: "json",
			success: function (data) {
			//var temp = data[data.legnth-1].timestamp;
			x=data[data.length-1].timestamp;
			categories.push(x);
			chart.xAxis[0].setCategories(categories);
			y=1*data[data.length-1].rpm;
console.log(y,1*data[data.length-1].rpm);
z=1*data[data.length-1].temp_max;
a=1*(data[data.length-1].B_vol_int+"."+data[data.length-1].B_vol_div);
b=1*data[data.length-1].motor_temp;
c=1*data[data.length-1].motor_torq;
			//if(temp!=data[data.length-1].timestamp){
			chart.redraw();
			console.log(data[data.length-1].timestamp);
			series.addPoint({y:z},true,true,true);
			series2.addPoint({y:a}, true, true,true);
			series3.addPoint({y:b}, true, true,true);
			series4.addPoint({y:c}, true, true,true);
			series5.addPoint({y:y}, true, true,true);
			//temp=data[data.length-1].timestamp;
//}
}})
                }, 3000);
}
	
}
       },
	title: {
	    text: 'Fourtor Data',
	style: {
            color: '#ffffff',
            fontWeight: 'bold',
            fontSize: '22px',
            fontFamily: 'Trebuchet MS, Verdana, sans-serif'
         },    
        },
        xAxis: {
	categories:[<?php echo join($dbdata,',') ?>] ,
	labels:{
	style:{
	color:'#fff',
	fontSize: '10px',
	},
	endabled:true,
        format:'{value:%M:%S}',
	},
	type: 'datetime',
	tickInterval: 1,
        },
	yAxis: {
            title: {
		text: 'Data Value',
	 style: {
            color: '#ffffff',
            fontWeight: 'bold',
            fontSize: '17px',
            fontFamily: 'Trebuchet MS, Verdana, sans-serif'
         },
	    },
	gridLineColor:'#bdc8da',
	labels:{
        style:{
        color:'#fff',
        fontSize: '10px',
        },
        },
        },
        series: [{
            name: 'Battery_temp',
		    data: [<?php echo join($Battery_temp,',')?>],//DB이름
                color:'#7595BC',
        }, {
            name: 'Battery_vol',
		    data: [<?php echo join($Battery_vol,',')?>],
                color:'#2B8F8E',
	},{
		name:'Motor_temp',
			data:[<?php echo join($Motor_temp,',')?>],
			color:'#F45B5B',	
	},{
		name:'Motor_torq',
		data:[<?php echo join($Motor_torq,',')?>],
		color:'#90EE7E',
	},{
		name:'RPM',
		data:[<?php echo join($rpm,',')?>],
		color:'#ebefaa',
	}],
            legend:{
                    floating:true,
                    align:'right',
                    verticalAlign:'top',
                    symbolRadius:5,
                    symbolwidth:10,
                    symbolHeight:10,
                    itemDistance:17,
                    itemStyle:{
color:'#fff',
                            fontSize:'14px',
                            fontWeight:'400',
                    },
                    x:10,
                    y:-3,
            },
    });
});
</script>
</head>
<body>
<!-- Highchart 그래프 출력 -->
<div id="container" style="width:100%; height:670px;background-image:url('real.png'); background-repeat: no-repeat; background-attachment:fixed; background-size:100% 100%; background-position:center top;">
</div>
</body>
</html>
