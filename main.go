package main

import (
		"math"
		"container/heap"
)

// Constants for direction and game phases
const (
		Up    = "up"
		Down  = "down"
		Left  = "left"
		Right = "right"

		EarlyGame = "early"
		MidGame   = "mid"
		LateGame  = "late"
)

// A* pathfinding

type Node struct {
		coord    Coord
		g, h, f  int
		parent   *Node
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].f < pq[j].f }
func (pq PriorityQueue) Swap(i, j int) { pq[i], pq[j] = pq[j], pq[i] }
func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*Node)) }
func (pq *PriorityQueue) Pop() interface{} {
		old := *pq
		n := len(old)
		item := old[n-1]
		*pq = old[0 : n-1]
		return item
}

func aStar(start, goal Coord, state GameState) []Coord {
		openList := &PriorityQueue{}
		heap.Init(openList)
		closedSet := make(map[Coord]bool)

		startNode := &Node{coord: start, g: 0, h: manhattanDistance(start, goal), f: 0}
		heap.Push(openList, startNode)

		for openList.Len() > 0 {
				current := heap.Pop(openList).(*Node)

				if current.coord == goal {
						path := []Coord{}
						for current != nil {
								path = append([]Coord{current.coord}, path...)
								current = current.parent
						}
						return path
				}

				closedSet[current.coord] = true

				for _, neighbor := range getAdjacentCoords(current.coord) {
						if !isOnBoard(neighbor, state.Board) || closedSet[neighbor] {
								continue
						}

						g := current.g + 1
						h := manhattanDistance(neighbor, goal)
						f := g + h

						existingNode := findNode(openList, neighbor)
						if existingNode == nil {
								newNode := &Node{coord: neighbor, g: g, h: h, f: f, parent: current}
								heap.Push(openList, newNode)
						} else if g < existingNode.g {
								existingNode.g = g
								existingNode.f = f
								existingNode.parent = current
								heap.Fix(openList, findIndex(openList, existingNode))
						}
				}
		}

		return nil
}

func findNode(pq *PriorityQueue, coord Coord) *Node {
		for _, node := range *pq {
				if node.coord == coord {
						return node
				}
		}
		return nil
}

func findIndex(pq *PriorityQueue, node *Node) int {
		for i, n := range *pq {
				if n == node {
						return i
				}
		}
		return -1
}

func manhattanDistance(a, b Coord) int {
		return int(math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y)))
}

// State machine for game phases

func getGamePhase(state GameState) string {
	boardSize := state.Board.Width * state.Board.Height
	midGameThreshold := int(float64(boardSize) * 0.3) // 30% of board size
	lateGameThreshold := int(float64(boardSize) * 0.5) // 50% of board size

	longestSnakeLength := 0
	for _, snake := range state.Board.Snakes {
		if snake.Length > longestSnakeLength {
			longestSnakeLength = snake.Length
		}
	}

	if longestSnakeLength < midGameThreshold {
		return EarlyGame
	} else if longestSnakeLength < lateGameThreshold {
		return MidGame
	} else {
		return LateGame
	}
}

// Advanced snake prediction

func predictSnakeMoves(state GameState) map[string][]Coord {
		predictions := make(map[string][]Coord)
		for _, snake := range state.Board.Snakes {
				if snake.ID == state.You.ID {
						continue
				}
				safeMoves := []Coord{}
				for _, coord := range getAdjacentCoords(snake.Head) {
						if isOnBoard(coord, state.Board) && !willCollide(coord, state, snake) {
								safeMoves = append(safeMoves, coord)
						}
				}
				if len(safeMoves) == 0 {
						safeMoves = append(safeMoves, snake.Head) // Stay in place if no safe moves
				}
				predictions[snake.ID] = safeMoves
		}
		return predictions
}

func willCollide(coord Coord, state GameState, snake Battlesnake) bool {
		for _, otherSnake := range state.Board.Snakes {
				if otherSnake.ID == snake.ID {
						continue
				}
				for i, body := range otherSnake.Body {
						if i == len(otherSnake.Body)-1 && len(otherSnake.Body) > len(snake.Body) {
								continue // Ignore tail of longer snakes
						}
						if coord == body {
								return true
						}
				}
		}
		return false
}

// Main move function with improved scoring

func move(state GameState) BattlesnakeMoveResponse {
		possibleMoves := []string{Up, Down, Left, Right}
		safeMoves := []string{}
		head := state.You.Head

		for _, move := range possibleMoves {
				if isSafeMove(head, move, state) {
						safeMoves = append(safeMoves, move)
				}
		}

		if len(safeMoves) == 0 {
				return BattlesnakeMoveResponse{Move: Down}
		}

		gamePhase := getGamePhase(state)
		predictions := predictSnakeMoves(state)

		moveScores := make(map[string]float64)
		for _, move := range safeMoves {
				newHead := getNextPosition(head, move)
				score := scoreMove(newHead, state, gamePhase, predictions)
				moveScores[move] = score
		}

		bestMove := safeMoves[0]
		bestScore := moveScores[bestMove]
		for _, move := range safeMoves[1:] {
				if moveScores[move] > bestScore {
						bestMove = move
						bestScore = moveScores[move]
				}
		}

		return BattlesnakeMoveResponse{
				Move: bestMove,
		}
}

func getNextPosition(head Coord, move string) Coord {
		switch move {
		case Up:
				return Coord{X: head.X, Y: head.Y + 1}
		case Down:
				return Coord{X: head.X, Y: head.Y - 1}
		case Left:
				return Coord{X: head.X - 1, Y: head.Y}
		case Right:
				return Coord{X: head.X + 1, Y: head.Y}
		}
		return head
}

func scoreMove(newHead Coord, state GameState, gamePhase string, predictions map[string][]Coord) float64 {
		score := 0.0

		// Calculate available space
		availableSpace := floodFill(newHead, state)
		score += float64(availableSpace) * 2  // Adjust this multiplier as needed

		// Food-seeking behavior
		if gamePhase == EarlyGame || state.You.Health < 50 {
				nearestFood := findNearestFood(newHead, state.Board.Food)
				if nearestFood != nil {
						path := aStar(newHead, *nearestFood, state)
						if path != nil {
								score += 100.0 / float64(len(path))
						}
				}
		}

		// Avoid predicted enemy positions
		for _, predictedMoves := range predictions {
				for _, predictedMove := range predictedMoves {
						if newHead == predictedMove {
								score -= 50
						}
				}
		}

		// Late game behavior: try to cut off other snakes
		if gamePhase == LateGame {
				for _, snake := range state.Board.Snakes {
						if snake.ID != state.You.ID {
								if isPositionCuttingOff(newHead, snake, state) {
										score += 30
								}
						}
				}
		}

		return score
}
func floodFill(start Coord, state GameState) int {
	visited := make(map[Coord]bool)
	queue := []Coord{start}
	space := 0

	for len(queue) > 0 {
			current := queue[0]
			queue = queue[1:]

			if visited[current] {
					continue
			}

			visited[current] = true
			space++

			for _, neighbor := range getAdjacentCoords(current) {
					if isOnBoard(neighbor, state.Board) && !isOccupied(neighbor, state) && !visited[neighbor] {
							queue = append(queue, neighbor)
					}
			}
	}

	return space
}

func isOccupied(coord Coord, state GameState) bool {
	for _, snake := range state.Board.Snakes {
			for _, body := range snake.Body {
					if coord == body {
							return true
					}
			}
	}
	return false
}

func findNearestFood(head Coord, food []Coord) *Coord {
	if len(food) == 0 {
			return nil
	}
	nearest := food[0]
	minDist := manhattanDistance(head, nearest)
	for _, f := range food[1:] {
			dist := manhattanDistance(head, f)
			if dist < minDist {
					nearest = f
					minDist = dist
			}
	}
	return &nearest
}

func isPositionCuttingOff(pos Coord, snake Battlesnake, state GameState) bool {
	snakeHead := snake.Head
	snakeTail := snake.Body[len(snake.Body)-1]

	// Check if the position is between the snake's head and tail
	if (pos.X >= snakeHead.X && pos.X <= snakeTail.X) || (pos.X <= snakeHead.X && pos.X >= snakeTail.X) {
			if (pos.Y >= snakeHead.Y && pos.Y <= snakeTail.Y) || (pos.Y <= snakeHead.Y && pos.Y >= snakeTail.Y) {
					return true
			}
	}
	return false
}

// Helper functions (make sure these are included in your implementation)

func isOnBoard(coord Coord, board Board) bool {
	return coord.X >= 0 && coord.X < board.Width && coord.Y >= 0 && coord.Y < board.Height
}

func getAdjacentCoords(coord Coord) []Coord {
	return []Coord{
			{X: coord.X, Y: coord.Y + 1}, // Up
			{X: coord.X, Y: coord.Y - 1}, // Down
			{X: coord.X - 1, Y: coord.Y}, // Left
			{X: coord.X + 1, Y: coord.Y}, // Right
	}
}

func getDirection(from, to Coord) string {
	if from.X < to.X {
			return Right
	} else if from.X > to.X {
			return Left
	} else if from.Y < to.Y {
			return Up
	} else if from.Y > to.Y {
			return Down
	}
	return ""
}

func isSafeMove(head Coord, direction string, state GameState) bool {
	newHead := getNextPosition(head, direction)

	if !isOnBoard(newHead, state.Board) {
			return false
	}

	for _, snake := range state.Board.Snakes {
			for i, bodyPart := range snake.Body {
					if i == len(snake.Body)-1 && snake.Length > state.You.Length {
							continue // Ignore tail of longer snakes
					}
					if newHead == bodyPart {
							return false
					}
			}
			if snake.ID != state.You.ID {
					for _, adjacentCoord := range getAdjacentCoords(snake.Head) {
							if newHead == adjacentCoord && snake.Length >= state.You.Length {
									return false
							}
					}
			}
	}

	return true
}

// Main functions

func info() BattlesnakeInfoResponse {
	return BattlesnakeInfoResponse{
			APIVersion: "1",
			Author:     "YourName",
			Color:      "#888888",
			Head:       "default",
			Tail:       "default",
	}
}

func start(state GameState) {
	// No initialization needed
}

func end(state GameState) {
	// No cleanup needed
}