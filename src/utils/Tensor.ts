/**
 * 3x1 column vector (tensor)
 */
export class Tensor3x1 {
  private data: number[] = [0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  clone(): Tensor3x1 {
    const result = new Tensor3x1();
    result.data = [...this.data];
    return result;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 1x3 row vector (tensor)
 */
export class Tensor1x3 {
  private data: number[] = [0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  /**
   * Multiply 1x3 row vector by 3x1 column vector to get scalar
   */
  mult(v: Tensor3x1): number {
    return this.data[0] * v.get(0) + this.data[1] * v.get(1) + this.data[2] * v.get(2);
  }

  /**
   * Multiply 1x3 row vector by 3x3 matrix to get 1x3 row vector
   */
  multMatrix(m: Tensor3x3): Tensor1x3 {
    const result = new Tensor1x3();
    for (let j = 0; j < 3; j++) {
      let sum = 0;
      for (let k = 0; k < 3; k++) {
        sum += this.data[k] * m.get(k, j);
      }
      result.setAt(j, sum);
    }
    return result;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 3x3 matrix (tensor)
 */
export class Tensor3x3 {
  private data: number[][] = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
  ];

  get(row: number, col: number): number {
    return this.data[row][col];
  }

  set(row: number, col: number, value: number): void {
    this.data[row][col] = value;
  }

  setIdentity(): void {
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        this.data[i][j] = i === j ? 1 : 0;
      }
    }
  }

  /**
   * Multiply 3x3 matrix by 3x1 column vector
   */
  mult(v: Tensor3x1): Tensor3x1 {
    const result = new Tensor3x1();
    for (let i = 0; i < 3; i++) {
      let sum = 0;
      for (let j = 0; j < 3; j++) {
        sum += this.data[i][j] * v.get(j);
      }
      result.setAt(i, sum);
    }
    return result;
  }

  /**
   * Multiply two 3x3 matrices
   */
  multMatrix(other: Tensor3x3): Tensor3x3 {
    const result = new Tensor3x3();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let k = 0; k < 3; k++) {
          sum += this.data[i][k] * other.get(k, j);
        }
        result.set(i, j, sum);
      }
    }
    return result;
  }

  /**
   * Add two 3x3 matrices
   */
  add(other: Tensor3x3): Tensor3x3 {
    const result = new Tensor3x3();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        result.set(i, j, this.data[i][j] + other.get(i, j));
      }
    }
    return result;
  }

  /**
   * Subtract two 3x3 matrices
   */
  subtract(other: Tensor3x3): Tensor3x3 {
    const result = new Tensor3x3();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        result.set(i, j, this.data[i][j] - other.get(i, j));
      }
    }
    return result;
  }

  /**
   * Compute outer product of 3x1 column vector with 1x3 row vector to get 3x3 matrix
   */
  static outerProduct(col: Tensor3x1, row: Tensor1x3): Tensor3x3 {
    const result = new Tensor3x3();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        result.set(i, j, col.get(i) * row.get(j));
      }
    }
    return result;
  }

  /**
   * Scale matrix by scalar
   */
  scale(scalar: number): Tensor3x3 {
    const result = new Tensor3x3();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        result.set(i, j, this.data[i][j] * scalar);
      }
    }
    return result;
  }

  toString(): string {
    return this.data.map(row => `[${row.join(', ')}]`).join('\n');
  }
}

/**
 * 4x1 column vector (tensor) for 4D state
 */
export class Tensor4x1 {
  private data: number[] = [0, 0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number, i3: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
    this.data[3] = i3;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  clone(): Tensor4x1 {
    const result = new Tensor4x1();
    result.data = [...this.data];
    return result;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 1x4 row vector (tensor)
 */
export class Tensor1x4 {
  private data: number[] = [0, 0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number, i3: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
    this.data[3] = i3;
  }

  multiply(vec: Tensor4x1): number {
    return this.data[0] * vec.get(0) +
           this.data[1] * vec.get(1) +
           this.data[2] * vec.get(2) +
           this.data[3] * vec.get(3);
  }

  multiplyMatrix(m: Tensor4x4): Tensor1x4 {
    const result = new Tensor1x4();
    for (let j = 0; j < 4; j++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += this.data[k] * m.get(k, j);
      }
      result.setAt(j, sum);
    }
    return result;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 4x4 matrix (tensor)
 */
export class Tensor4x4 {
  private data: number[][] = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
  ];

  get(i: number, j: number): number {
    return this.data[i][j];
  }

  set(i: number, j: number, value: number): void {
    this.data[i][j] = value;
  }

  multiply(vec: Tensor4x1): Tensor4x1 {
    const result = new Tensor4x1();
    for (let i = 0; i < 4; i++) {
      let sum = 0;
      for (let j = 0; j < 4; j++) {
        sum += this.data[i][j] * vec.get(j);
      }
      result.setAt(i, sum);
    }
    return result;
  }

  multiplyMatrix(other: Tensor4x4): Tensor4x4 {
    const result = new Tensor4x4();
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        let sum = 0;
        for (let k = 0; k < 4; k++) {
          sum += this.data[i][k] * other.get(k, j);
        }
        result.set(i, j, sum);
      }
    }
    return result;
  }

  add(other: Tensor4x4): Tensor4x4 {
    const result = new Tensor4x4();
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        result.set(i, j, this.data[i][j] + other.get(i, j));
      }
    }
    return result;
  }

  scale(scalar: number): Tensor4x4 {
    const result = new Tensor4x4();
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        result.set(i, j, this.data[i][j] * scalar);
      }
    }
    return result;
  }

  clone(): Tensor4x4 {
    const result = new Tensor4x4();
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        result.set(i, j, this.data[i][j]);
      }
    }
    return result;
  }

  toString(): string {
    return this.data.map(row => `[${row.join(', ')}]`).join('\n');
  }
}

/**
 * 5x1 column vector (tensor) for extended state
 */
export class Tensor5x1 {
  private data: number[] = [0, 0, 0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number, i3: number, i4: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
    this.data[3] = i3;
    this.data[4] = i4;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  clone(): Tensor5x1 {
    const result = new Tensor5x1();
    result.data = [...this.data];
    return result;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 1x5 row vector (tensor)
 */
export class Tensor1x5 {
  private data: number[] = [0, 0, 0, 0, 0];

  get(index: number): number {
    return this.data[index];
  }

  set(i0: number, i1: number, i2: number, i3: number, i4: number): void {
    this.data[0] = i0;
    this.data[1] = i1;
    this.data[2] = i2;
    this.data[3] = i3;
    this.data[4] = i4;
  }

  setAt(index: number, value: number): void {
    this.data[index] = value;
  }

  /**
   * Multiply row vector (1x5) by column vector (5x1) to get scalar
   */
  multiply(col: Tensor5x1): number {
    let result = 0;
    for (let i = 0; i < 5; i++) {
      result += this.data[i] * col.get(i);
    }
    return result;
  }

  /**
   * Multiply row vector (1x5) by 5x5 matrix to get row vector (1x5)
   */
  multiplyMatrix(matrix: Tensor5x5): Tensor1x5 {
    const result = new Tensor1x5();
    for (let j = 0; j < 5; j++) {
      let sum = 0;
      for (let k = 0; k < 5; k++) {
        sum += this.data[k] * matrix.get(k, j);
      }
      result.setAt(j, sum);
    }
    return result;
  }

  toString(): string {
    return `[${this.data.join(', ')}]`;
  }
}

/**
 * 5x5 matrix (tensor)
 */
export class Tensor5x5 {
  private data: number[][] = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
  ];

  get(row: number, col: number): number {
    return this.data[row][col];
  }

  set(row: number, col: number, value: number): void {
    this.data[row][col] = value;
  }

  /**
   * Multiply 5x5 matrix by 5x1 column vector to get 5x1 column vector
   */
  multiply(col: Tensor5x1): Tensor5x1 {
    const result = new Tensor5x1();
    for (let i = 0; i < 5; i++) {
      let sum = 0;
      for (let j = 0; j < 5; j++) {
        sum += this.data[i][j] * col.get(j);
      }
      result.setAt(i, sum);
    }
    return result;
  }

  /**
   * Multiply two 5x5 matrices
   */
  multiplyMatrix(other: Tensor5x5): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        let sum = 0;
        for (let k = 0; k < 5; k++) {
          sum += this.data[i][k] * other.get(k, j);
        }
        result.set(i, j, sum);
      }
    }
    return result;
  }

  /**
   * Add two 5x5 matrices
   */
  add(other: Tensor5x5): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        result.set(i, j, this.data[i][j] + other.get(i, j));
      }
    }
    return result;
  }

  /**
   * Subtract two 5x5 matrices
   */
  subtract(other: Tensor5x5): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        result.set(i, j, this.data[i][j] - other.get(i, j));
      }
    }
    return result;
  }

  /**
   * Compute outer product of 5x1 column vector with 1x5 row vector to get 5x5 matrix
   */
  static outerProduct(col: Tensor5x1, row: Tensor1x5): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        result.set(i, j, col.get(i) * row.get(j));
      }
    }
    return result;
  }

  /**
   * Scale matrix by scalar
   */
  scale(scalar: number): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        result.set(i, j, this.data[i][j] * scalar);
      }
    }
    return result;
  }

  /**
   * Clone this matrix
   */
  clone(): Tensor5x5 {
    const result = new Tensor5x5();
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        result.set(i, j, this.data[i][j]);
      }
    }
    return result;
  }

  toString(): string {
    return this.data.map(row => `[${row.join(', ')}]`).join('\n');
  }
}
