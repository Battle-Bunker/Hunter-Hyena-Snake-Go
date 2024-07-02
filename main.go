package main

import (
	"encoding/json"
	"log"
	"math"
	"net/http"
)

type Coord struct {
	X int `json:"x"`
	Y int `json:"y"`
}

type Battlesnake struct {
	ID     string  `json:"id"`
	Name   string  `json:"name"`
	Health int     `json:"health"`
	Body   []Coord `json:"body"`
	Head   Coord   `json:"head"`
	Length int     `json:"length"`
}

type Board struct {
	Height int           `json:"height"`
	Width  int           `json:"width"`
	Food   []Coord       `json:"food"`
	Snakes []Battlesnake `json:"snakes"`
}

type GameState struct {
	Game  map[string]interface{} `json:"game"`
	Turn  int                    `json:"turn"`
	Board Board                  `json:"board"`
	You   Battlesnake            `json:"you"`
}

func main() {
	http.HandleFunc("/", handleIndex)
	http.HandleFunc("/start", handleStart)
	http.HandleFunc("/move", handleMove)
	http.HandleFunc("/end", handleEnd)

	log.Fatal(http.ListenAndServe(":8080", nil))
}

func handleIndex(w http.ResponseWriter, r *http.Request) {
	response := map[string]string{
		"apiversion": "1",
		"author":     "YourName",
		"color":      "#FF0000",
		"head":       "default",
		"tail":       "default",
	}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func handleStart(w http.ResponseWriter, r *http.Request) {
	w.WriteHeader(http.StatusOK)
}

func handleMove(w http.ResponseWriter, r *http.Request) {
	state := GameState{}
	err := json.NewDecoder(r.Body).Decode(&state)
	if err != nil {
		log.Printf("ERROR: Failed to decode move json: %s", err)
		return
	}

	move := getBestMove(state)

	response := map[string]string{"move": move}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func handleEnd(w http.ResponseWriter, r *http.Request) {
	w.WriteHeader(http.StatusOK)
}

func getBestMove(state GameState) string {
	possibleMoves := []string{"up", "down", "left", "right"}
	myHead := state.You.Head

	// Check for immediate collisions
	safetyMoves := []string{}
	for _, move := range possibleMoves {
		nextPos := getNextPosition(myHead, move)
		if isSafe(nextPos, state) {
			safetyMoves = append(safetyMoves, move)
		}
	}

	if len(safetyMoves) == 0 {
		return "up" // No safe moves, just go up
	}

	// Find closest food
	closestFood := findClosestFood(myHead, state.Board.Food)

	// Find smaller snakes
	smallerSnakes := findSmallerSnakes(state.You, state.Board.Snakes)

	// Prioritize food if health is low or no smaller snakes nearby
	if state.You.Health < 50 || len(smallerSnakes) == 0 {
		return moveTowards(myHead, closestFood, safetyMoves)
	}

	// Find closest smaller snake
	closestSmallSnake := findClosestSnake(myHead, smallerSnakes)

	// Decide whether to go for food or attack
	if distance(myHead, closestFood) < distance(myHead, closestSmallSnake.Head) {
		return moveTowards(myHead, closestFood, safetyMoves)
	} else {
		return moveTowards(myHead, closestSmallSnake.Head, safetyMoves)
	}
}

func isSafe(pos Coord, state GameState) bool {
	// Check board boundaries
	if pos.X < 0 || pos.Y < 0 || pos.X >= state.Board.Width || pos.Y >= state.Board.Height {
		return false
	}

	// Check for collision with any snake body
	for _, snake := range state.Board.Snakes {
		for _, bodyPart := range snake.Body {
			if pos.X == bodyPart.X && pos.Y == bodyPart.Y {
				return false
			}
		}
	}

	return true
}

func getNextPosition(head Coord, move string) Coord {
	switch move {
	case "up":
		return Coord{head.X, head.Y + 1}
	case "down":
		return Coord{head.X, head.Y - 1}
	case "left":
		return Coord{head.X - 1, head.Y}
	case "right":
		return Coord{head.X + 1, head.Y}
	}
	return head
}

func findClosestFood(head Coord, foods []Coord) Coord {
	closestFood := foods[0]
	minDist := math.MaxFloat64

	for _, food := range foods {
		dist := distance(head, food)
		if dist < minDist {
			minDist = dist
			closestFood = food
		}
	}

	return closestFood
}

func findSmallerSnakes(you Battlesnake, snakes []Battlesnake) []Battlesnake {
	smallerSnakes := []Battlesnake{}
	for _, snake := range snakes {
		if snake.ID != you.ID && snake.Length < you.Length {
			smallerSnakes = append(smallerSnakes, snake)
		}
	}
	return smallerSnakes
}

func findClosestSnake(head Coord, snakes []Battlesnake) Battlesnake {
	closestSnake := snakes[0]
	minDist := math.MaxFloat64

	for _, snake := range snakes {
		dist := distance(head, snake.Head)
		if dist < minDist {
			minDist = dist
			closestSnake = snake
		}
	}

	return closestSnake
}

func distance(a, b Coord) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

func moveTowards(from, to Coord, safeMoves []string) string {
	dx := to.X - from.X
	dy := to.Y - from.Y

	preferredMoves := []string{}

	if math.Abs(float64(dx)) > math.Abs(float64(dy)) {
		if dx > 0 {
			preferredMoves = append(preferredMoves, "right")
		} else {
			preferredMoves = append(preferredMoves, "left")
		}
		if dy > 0 {
			preferredMoves = append(preferredMoves, "up")
		} else {
			preferredMoves = append(preferredMoves, "down")
		}
	} else {
		if dy > 0 {
			preferredMoves = append(preferredMoves, "up")
		} else {
			preferredMoves = append(preferredMoves, "down")
		}
		if dx > 0 {
			preferredMoves = append(preferredMoves, "right")
		} else {
			preferredMoves = append(preferredMoves, "left")
		}
	}

	for _, move := range preferredMoves {
		for _, safeMove := range safeMoves {
			if move == safeMove {
				return move
			}
		}
	}

	return safeMoves[0] // If no preferred move is safe, return the first safe move
}
func getBestMove(state GameState) string {
	possibleMoves := []string{"up", "down", "left", "right"}
	myHead := state.You.Head

	// Check for immediate collisions and evaluate space
	safetyMoves := []string{}
	spaceCounts := make(map[string]int)
	for _, move := range possibleMoves {
		nextPos := getNextPosition(myHead, move)
		if isSafe(nextPos, state) {
			safetyMoves = append(safetyMoves, move)
			spaceCounts[move] = floodFill(nextPos, state)
		}
	}

	if len(safetyMoves) == 0 {
		return "up" // No safe moves, just go up
	}

	// Find closest food
	closestFood := findClosestFood(myHead, state.Board.Food)

	// Find smaller snakes
	smallerSnakes := findSmallerSnakes(state.You, state.Board.Snakes)

	// Endgame strategy
	if len(state.Board.Snakes) == 2 {
		return endgameStrategy(state, safetyMoves, spaceCounts)
	}

	// Prioritize food if health is low or no smaller snakes nearby
	if state.You.Health < 50 || len(smallerSnakes) == 0 {
		return moveTowardsWithSpace(myHead, closestFood, safetyMoves, spaceCounts)
	}

	// Find best target snake
	targetSnake := findBestTargetSnake(state.You, smallerSnakes, state.Board.Food)

	// Decide whether to go for food or attack
	if shouldGoForFood(state.You, targetSnake, closestFood) {
		return moveTowardsWithSpace(myHead, closestFood, safetyMoves, spaceCounts)
	} else {
		return interceptSnake(state.You, targetSnake, safetyMoves, spaceCounts)
	}
}

func floodFill(start Coord, state GameState) int {
	visited := make(map[Coord]bool)
	queue := []Coord{start}
	count := 0

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		if visited[current] || !isSafe(current, state) {
			continue
		}

		visited[current] = true
		count++

		for _, move := range []string{"up", "down", "left", "right"} {
			next := getNextPosition(current, move)
			queue = append(queue, next)
		}
	}

	return count
}

func moveTowardsWithSpace(from, to Coord, safeMoves []string, spaceCounts map[string]int) string {
	preferredMoves := getPreferredMoves(from, to)

	bestMove := ""
	maxSpace := -1

	for _, move := range preferredMoves {
		if contains(safeMoves, move) && spaceCounts[move] > maxSpace {
			bestMove = move
			maxSpace = spaceCounts[move]
		}
	}

	if bestMove != "" {
		return bestMove
	}

	return safeMoves[0] // If no preferred move is safe, return the first safe move
}

func endgameStrategy(state GameState, safeMoves []string, spaceCounts map[string]int) string {
	otherSnake := state.Board.Snakes[0]
	if otherSnake.ID == state.You.ID {
		otherSnake = state.Board.Snakes[1]
	}

	if state.You.Length > otherSnake.Length {
		// If we're longer, try to cut off the other snake
		return interceptSnake(state.You, otherSnake, safeMoves, spaceCounts)
	} else {
		// If we're shorter or equal, prioritize space and food
		closestFood := findClosestFood(state.You.Head, state.Board.Food)
		return moveTowardsWithSpace(state.You.Head, closestFood, safeMoves, spaceCounts)
	}
}

func findBestTargetSnake(you Battlesnake, smallerSnakes []Battlesnake, foods []Coord) Battlesnake {
	var bestTarget Battlesnake
	minScore := math.MaxFloat64

	for _, snake := range smallerSnakes {
		distToYou := distance(you.Head, snake.Head)
		distToFood := distance(snake.Head, findClosestFood(snake.Head, foods))
		score := distToYou + 0.5*distToFood // Balance between closeness to you and food

		if score < minScore {
			minScore = score
			bestTarget = snake
		}
	}

	return bestTarget
}

func shouldGoForFood(you Battlesnake, targetSnake Battlesnake, closestFood Coord) bool {
	distToFood := distance(you.Head, closestFood)
	distToSnake := distance(you.Head, targetSnake.Head)

	return you.Health < 50 || distToFood < distToSnake*0.75
}

func interceptSnake(you Battlesnake, target Battlesnake, safeMoves []string, spaceCounts map[string]int) string {
	predictedPos := predictSnakeMove(target, you.Head)
	return moveTowardsWithSpace(you.Head, predictedPos, safeMoves, spaceCounts)
}

func predictSnakeMove(snake Battlesnake, yourHead Coord) Coord {
	possibleMoves := []Coord{
		{snake.Head.X, snake.Head.Y + 1},
		{snake.Head.X, snake.Head.Y - 1},
		{snake.Head.X - 1, snake.Head.Y},
		{snake.Head.X + 1, snake.Head.Y},
	}

	bestMove := snake.Head
	maxDist := -1.0

	for _, move := range possibleMoves {
		dist := distance(move, yourHead)
		if dist > maxDist {
			maxDist = dist
			bestMove = move
		}
	}

	return bestMove
}

func getPreferredMoves(from, to Coord) []string {
	dx := to.X - from.X
	dy := to.Y - from.Y

	preferredMoves := []string{}

	if math.Abs(float64(dx)) > math.Abs(float64(dy)) {
		if dx > 0 {
			preferredMoves = append(preferredMoves, "right")
		} else {
			preferredMoves = append(preferredMoves, "left")
		}
		if dy > 0 {
			preferredMoves = append(preferredMoves, "up")
		} else {
			preferredMoves = append(preferredMoves, "down")
		}
	} else {
		if dy > 0 {
			preferredMoves = append(preferredMoves, "up")
		} else {
			preferredMoves = append(preferredMoves, "down")
		}
		if dx > 0 {
			preferredMoves = append(preferredMoves, "right")
		} else {
			preferredMoves = append(preferredMoves, "left")
		}
	}

	return preferredMoves
}

func contains(slice []string, item string) bool {
	for _, a := range slice {
		if a == item {
			return true
		}
	}
	return false
}