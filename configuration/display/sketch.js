
let w;
let columns;
let rows;
let board;
let next;
let init_state;
let fr = 8;

function Cell(locationX, locationY){
  this.posX = locationX,
  this.posY = locationY,
  this.move = function(){
    var randomX = Math.floor(Math.random() * 3 -1);
    var randomy = Math.floor(Math.random() * 3 -1);
    this.posX += randomX;
    this.posY += randomy;
    if(this.posX > columns){
      this.posX = columns
    }else if(this.posX < 0){
      this.posX = 0
    }
    if(this.posY > rows){
      this.posY = rows
    }else if(this.posY < 0){
      this.posY = 0
    }
  }

};


function setup() {
  createCanvas(720, 400);
  frameRate(fr);
  w = 20;
  // Calculate columns and rows
  init_state = false;
  columns = floor(width / w);
  rows = floor(height / w);
  // Wacky way to make a 2D array is JS
  board = new Array(columns);
  for (let i = 0; i < columns; i++) {
    board[i] = new Array(rows);
  }
  // Going to use multiple 2D arrays and swap them
  next = new Array(columns);
  for (i = 0; i < columns; i++) {
    next[i] = new Array(rows);
  }
  cells = [];
  cells.push(new Cell(20, 20))
  init();
}

function draw() {
  background(255);
  // generate();
  for (let i = 0; i < cells.length; i++){
    board[cells[i].posX][cells[i].posY] = 0;
    cells[i].move();
    board[cells[i].posX][cells[i].posY] = 1;
  };
  for ( let i = 0; i < columns;i++) {
    for ( let j = 0; j < rows;j++) {
      if ((board[i][j] == 1)) fill(0);
      else fill(255);
      stroke(0);
      rect(i * w, j * w, w-1, w-1);
    }
  }

}

// reset board when mouse is pressed
function mousePressed() {
  if(!init_state){
    init();
    init_state = true;
  }else{
  }
}

function bitFlip(){
  c = floor(mouseX / w);
  r = floor(mouseY / w);
  board[c][r] = !board[c][r];
};

// Fill board randomly
function init() {
  for (let i = 0; i < columns; i++) {
    for (let j = 0; j < rows; j++) {
      // Lining the edges with 0s
      if (i == 0 || j == 0 || i == columns-1 || j == rows-1) board[i][j] = 0;
      // Filling the rest randomly
      else board[i][j] = floor(random(2));
      next[i][j] = 0;
    }
  }
}

// // The process of creating the new generation
// function generate() {

//   // Loop through every spot in our 2D array and check spots neighbors
//   for (let x = 1; x < columns - 1; x++) {
//     for (let y = 1; y < rows - 1; y++) {
//       // Add up all the states in a 3x3 surrounding grid
//       let neighbors = 0;
//       for (let i = -1; i <= 1; i++) {
//         for (let j = -1; j <= 1; j++) {
//           neighbors += board[x+i][y+j];
//         }
//       }

//       // A little trick to subtract the current cell's state since
//       // we added it in the above loop
//       neighbors -= board[x][y];
//       // Rules of Life
//       if      ((board[x][y] == 1) && (neighbors <  2)) next[x][y] = 0;           // Loneliness
//       else if ((board[x][y] == 1) && (neighbors >  3)) next[x][y] = 0;           // Overpopulation
//       else if ((board[x][y] == 0) && (neighbors == 3)) next[x][y] = 1;           // Reproduction
//       else                                             next[x][y] = board[x][y]; // Stasis
//     }
//   }

//   // Swap!
//   let temp = board;
//   board = next;
//   next = temp;
// }

