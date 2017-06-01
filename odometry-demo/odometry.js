var __interpretation_started_timestamp__;
var pi = 3.141592653589793;
var i, j, k;

//Переменные для построения сантиметровой сетки и алгоритма А*
var scale = 100;
var xShift = 0, yShift = 0;
var h, w ;
var distanceCoefficient = 4;
var dx = [1, 1, 0, -1, -1, -1, 0, 1];
var dy = [0, 1, 1, 1, 0, -1, -1, -1];

var u;

var mLeft = brick.motor(M3);
var mRight = brick.motor(M4);

var eLeft = brick.encoder(E3);
var eRight = brick.encoder(E4);
var gyro = brick.gyroscope();
var infra = brick.sensor(A1);

// в этих переменных будут хранится данные с энкодеров
var encLeftOld = 0;
var encRightOld = 0;
var encLeft = 0;
var encRight = 0;

// переменные для подсчета текущих координат
var center = 0.0;
var tiltMdps = 0.0; // скорость изменения угла отклонения в миллиградусах в секунду
var tilt = 0.0;
var dtilt = 0.0;
var prevTilt = 0.0;
// текущие координаты
var ex = 0.0;
var ey = 0.0;
var grid = new Array(h);
var route = new Array(0);

//массив для записи координат
var roboWasAt = [[0.0, 0.0]];
//если counter == CONST_FOR_COUNTER, тогда записываем в roboWasAt текущие координаты
var counter = 0;
var CONST_FOR_COUNTER = 5;

//----------------------------------------------------------------------------------FUNCTIONS----------------

//выполняет инициализацию энкодеров и датчика линии
var initialisation = function()
{
    eLeft.reset();
    eRight.reset();
    brick.lineSensor("video0").detect();
    brick.gyroscope().calibrate(100);
}

// ждет команду от пользователя (нажатие галочки на панели)
var waitForStartCommand = function()
{
    brick.configure("video0", "lineSensor");
    brick.lineSensor("video0").init(true);

    brick.display().setBackground("red");
    brick.display().redraw();

    brick.display().addLabel("Press Enter", 40, 100);
    brick.display().redraw();

    brick.display().addLabel("when ready", 40, 120);
    brick.display().redraw();

	while (!brick.keys().wasPressed(KeysEnum.Enter)) {
	 script.wait(100);
	 }

    brick.display().clear();
    brick.display().redraw();

    brick.display().setBackground("green");
    brick.display().redraw();
}

function toRadians (angle)
{
    return angle * (pi / 180);
}

//Диагональное расстояние

function distance(a, b)
{
    var d1 = 1, d2 = Math.sqrt(2);
    var distX = Math.abs(a.x - b.x);
    var distY = Math.abs(a.y - b.y);
    return d1 * (distX + distY) + (d2 - 2 * d1)*Math.min(distX, distY);
}

//A*

function aStar(start, finish,initDir) {
    var from = new Array(h);
    var cost = new Array(h);
    for (i = 0; i < h; i++)
    {
        cost[i] = new Array(0);
        var costI = cost[i];
        from[i] = new Array(0);
        var fromI = from[i];
        for (j = 0; j<w; j++)
        {
            fromI.push(-2);
            costI.push(10000);
        }
    }
    print(grid[start.x][start.y]);
    cost[start.x][start.y] = 0;
    from[start.x][start.y] = -1;
    var queue = [{point:start, priority:0}];
    while (queue.length>0) {
        if (queue[0].point === finish) {
            break;
        }
        else {
            var front = queue.shift();
            var current = front.point;
            for (j = 0; j < 8; j++) {
                if (grid[current.x + dx[j]][current.y + dy[j]] > 0) {
                    var next = {x: current.x + dx[j], y: current.y + dy[j]};
                    var turnCost;
                    if (from[current.x][current.y] !== -1)
                        turnCost = Math.abs(from[current.x][current.y] - j);
                    else
                        turnCost = Math.abs(j - initDir);
                    if (turnCost > 4)
                        turnCost = 8 - turnCost;
                    turnCost = turnCost * turnCost * turnCost;
                    var newCost = cost[current.x][current.y] + turnCost + distance(current, next);
                    if (newCost < cost[next.x][next.y]) {
                        cost[next.x][next.y] = newCost;
                        queue.push({point: next, priority: newCost + distance(next, finish)});
                        from[next.x][next.y] = j;
                    }
                }
            }
        }
        queue.sort(function (a, b) {
            return a.priority - b.priority;
        });
    }
    var xRoute = finish.x, yRoute = finish.y, fromRoute = from[xRoute][yRoute];
    while (fromRoute !== -1)
    {
        route.unshift({x:(xRoute-xShift)*scale, y:(yRoute-yShift)*scale});
        xRoute = xRoute - dx[fromRoute];
        yRoute = yRoute - dy[fromRoute];
        fromRoute = from[xRoute][yRoute];
    }
    route.unshift({x:(xRoute-xShift)*scale, y:(yRoute-yShift)*scale});
}

//----------------------------------------------------------------------------------MAIN-----------------

var main = function()
{
    __interpretation_started_timestamp__ = Date.now();
    waitForStartCommand();
    initialisation();

    k = 1;

	 while (infra.read() > 50) {    //остановка по датчику расстояния
	 u = brick.lineSensor("video0").read()[0];
	 encLeft = eLeft.readRawData();
	 encRight = eRight.readRawData();

	 mLeft.setPower(30 + k * u);
	 mRight.setPower(30 - k * u);


	 tilt = gyro.read()[6] / 1000; // эта команда считывает угол отклонения от оси Х

	 //считаем текущие координаты, см презентацию по одометрии
	 var l = encLeft - encLeftOld;
	 var r =  encRight - encRightOld;
	 center = (l + r) / 2;
	 if (prevTilt + tilt < 0.1)
		 prevTilt = tilt;
	 dtilt = (prevTilt + tilt) / 2;
	 var rad = toRadians(dtilt);
	 ex = ex + Math.cos(rad) * center;
	 ey = ey + Math.sin(rad) * center;

	 encLeftOld = encLeft;
	 encRightOld = encRight;
	 prevTilt = tilt;

	 //сохранение полученных координат
	 counter += 1;
	 if (counter == CONST_FOR_COUNTER)
	 {
	 roboWasAt.push([ex, ey]);
	 counter = 0;
	 }



	 script.wait(200);

	 }

	 mLeft.setPower(0);
	 mRight.setPower(0);
    print("Start");

    //инициализация массива сетки
    var m = roboWasAt.length;

    var s1 = {x:0, y:0};
    var s2 = {x:0, y:0};
    var f1 = {x:0, y:0};
    var f2 = {x:0, y:0};
    var xMin = 0, xMax = 0, yMin = 0, yMax = 0;
    for (i = 0; i < m; i++) {
        var current = roboWasAt[i];
        var x = current[0];
        if (x > xMax) {
            xMax = x;
        }
        else {
            if (x < xMin) {
                xMin = x;
            }
        }
        var y = current[1];
        if (y > yMax) {
            yMax = y;
        }
        else {
            if (y < yMin) {
                yMin = y;
            }
        }
    }
    if (xMin < 0)
    {
        xShift = -Math.round(xMin / scale) + distanceCoefficient + 5;
    }
    h = Math.round(xMax / scale) + xShift + distanceCoefficient + 5;
    if (yMin < 0)
    {
        yShift = -Math.round(yMin / scale) + distanceCoefficient + 5;
    }
    w = Math.round(yMax / scale) + yShift + distanceCoefficient + 5;

    print("Shifts: ",xShift," ",yShift);
    print(h," ",w);

    for (i = 0; i < h; i++)
    {
        grid[i] = new Array(0);
        for (j = 0; j<w; j++)
        {
            grid[i].push(0);
        }
    }

    //расширение границ
    for (i = 0; i < m; i++) {
        var current = roboWasAt[i];
        var x = current[0];
        var a = Math.round(x/scale) + xShift;
        var y = current[1];
        var b = Math.round(y/scale) + yShift;
        if (i === 0) {
            s1 = {x: a, y: b};
        }
        if (i === Math.round(m/2)){
            s2 = {x:a, y: b};
        }

        if (i === Math.round(m/2) - 1){
            f1 = {x:a, y:b};
        }
        if (i === m-1){
            f2 = {x:a, y:b};
        }
        grid[a][b] = 1;
    }

    var allowedPoints = new Array(0);
    for (i = 0; i < h; i++) {
        var gridI = grid[i];
        for (j = 0; j < w; j++) {
            if (gridI[j] === 1)
                allowedPoints.push({x: i, y: j, d: distanceCoefficient});
        }
    }
    while (allowedPoints.length > 0)
    {
        var front = allowedPoints.shift();
        var dist = front.d, currentX = front.x, currentY = front.y;
        for (k = 0; k<4; k++)
        {
            grid[currentX][currentY] = dist;
            if ((grid[currentX + dx[2*k]][currentY + dy[2*k]] < dist) && (dist > 0))
            {
                allowedPoints.unshift({x: currentX + dx[2*k], y: currentY + dy[2*k], d: dist-1});
            }
        }
    }

    //Вызов алгоритма
    aStar(s2,f2,4);
    aStar(s1,f1,0);

    //Вывод траектории
    print("Route Start");
    for (i = 0; i<route.length; i++)
    {
        print(route[i].x, " ", route[i].y);
    }
    print("Route End");

    //Движение по точкам
    ex = -400;
    ey = 0;
    var n = 0;
    k = 100;
    eLeft.reset();
    eRight.reset();
    encLeftOld = 0;
    encRightOld = 0;
    prevTilt = 0;

    while (true)
    {
        var nextX = route[n].x, nextY = route[n].y;
        while (Math.abs(ex-nextX)>100 || Math.abs(ey-nextY)>100)
        {
            //считаем текущие координаты
            encLeft = eLeft.readRawData();
            encRight = eRight.readRawData();

            tilt = gyro.read()[6]/1000;

            var l = encLeft - encLeftOld;
            var r =  encRight - encRightOld;

            center = ( l + r) / 2;
            if (prevTilt + tilt < 0.1)
            	prevTilt = tilt;
            dtilt = (prevTilt + tilt) / 2;
            var rad = toRadians(dtilt);

            ex = ex + Math.cos(rad) * center;
            ey = ey + Math.sin(rad) * center;

            //считаем направление в цели и корректируем текущее направление
            var curRad = toRadians(tilt);
            var nextSin = (nextY - ey)/(Math.sqrt((nextY - ey)*(nextY - ey) + (nextX - ex)*(nextX - ex)));
            var nextCos = (nextX - ex)/(Math.sqrt((nextY - ey)*(nextY - ey) + (nextX - ex)*(nextX - ex)));
            var nextSign = 2*(nextSin > 0)-1;
            var nextRad = Math.acos(nextCos)*nextSign;
            u = nextRad - curRad;
            if (u > pi)
            {
                print("More");
                u = -2*pi + u;
            }
            if (u < -pi)
            {
                print("Less");
                u = 2*pi + u;
            }
            mLeft.setPower(60 + k*u);
            mRight.setPower(60 - k*u);

            encLeftOld = encLeft;
            encRightOld = encRight;
            prevTilt = tilt;

            script.wait(200);
        }

        n = n+1;
        if (n === route.length)
            n = 0;
    }

}

