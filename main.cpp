#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <vector>

struct Star
{
    sf::Vector2f position;
    float radius;
};

class Boid
{
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    sf::ConvexShape shape;

    Boid(float x, float y, std::mt19937 generator)
    {
        std::uniform_real_distribution<float> distribution(-2.f, 2.f);


        position = sf::Vector2f(x, y);
        velocity = sf::Vector2f(distribution(generator), distribution(generator));
        acceleration = sf::Vector2f(0.f, 0.f);

        const float sizeScale = generateSize(generator);
        sf::Vector2f newSize (15.f * sizeScale, 2.f * sizeScale);
        shape.setPointCount(3);
        shape.setPoint(0,sf::Vector2f(10, 0));
        shape.setPoint(1, sf::Vector2f(-5, -5));
        shape.setPoint(2, sf::Vector2f(-5, 5));
        shape.setOrigin(0, 0);
        shape.setScale(sizeScale, sizeScale);
        shape.setFillColor(sf::Color(generateColor(generator), generateColor(generator), generateColor(generator)));
        shape.setPosition(position);
    }

    int generateColor(std::mt19937& generator)
    {
        std::uniform_int_distribution<int> distribution(0, 255);
        return distribution(generator);
    }

    float generateSize(std::mt19937& generator)
    {
        std::uniform_real_distribution<float> distribution(0.8f, 1.2f);
        return distribution(generator);
    }

    void updateBoidPosition(float windowWidth, float windowHeight)
    {
        velocity += acceleration;

        const float velocityMagnitude = sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y));

        if (velocityMagnitude > 5.f)
        {
            velocity = (velocity / velocityMagnitude) * 5.f;
        }

        position += velocity;
        acceleration *= 0.f;

        float rotationAngle = atan2(velocity.y, velocity.x) * (180.f / M_PI);

        shape.setPosition(position);
        shape.setRotation(rotationAngle);

        if (position.x < 0)
        {
            position.x += windowWidth;
        }
        else if (position.x > windowWidth)
        {
            position.x -= windowWidth;
        }

        if (position.y < 0)
        {
            position.y += windowHeight;
        }
        else if (position.y > windowHeight)
        {
            position.y -= windowHeight;
        }
    }

    void flockRules(std::vector<Boid>& boids, sf::Vector2f mousePosition, bool blackHoleActive, sf::Vector2f blackHolePosition)
    {
        sf::Vector2f totalPosition = sf::Vector2f(0.f, 0.f);
        sf::Vector2f totalVelocity = sf::Vector2f(0.f, 0.f);

        const float mouseDx = mousePosition.x - this->position.x;
        const float mouseDy = mousePosition.y - this->position.y;
        float mouseDistance = sqrt((mouseDx * mouseDx) + (mouseDy * mouseDy));

        const float blackHoleDx = blackHolePosition.x - this->position.x;
        const float blackHoleDy = blackHolePosition.y - this->position.y;
        float blackHoleDistance = sqrt((blackHoleDx * blackHoleDx) + (blackHoleDy * blackHoleDy));

        if (blackHoleActive)
        {
            if (blackHoleDistance < 150.f)
            {
                sf::Vector2f attractForce = blackHolePosition - this->position;
                float magnitudeOfAttractForce = sqrt((attractForce.x * attractForce.x) + (attractForce.y * attractForce.y));

                if (magnitudeOfAttractForce > 0)
                {
                    sf::Vector2f normalisedAttractForce = attractForce / magnitudeOfAttractForce;
                    normalisedAttractForce *= 2.f;
                    this->acceleration += normalisedAttractForce;
                }
            }
        }

        if (mouseDistance < 100.f && blackHoleActive == false)
        {
            sf::Vector2f fleeDirection = this->position - mousePosition;
            float magnitudeOfFleeDirection = sqrt((fleeDirection.x * fleeDirection.x) + (fleeDirection.y * fleeDirection.y));

            if (magnitudeOfFleeDirection > 0)
            {
                sf::Vector2f normalisedFleeingdirection = fleeDirection / magnitudeOfFleeDirection;
                normalisedFleeingdirection *= 1.f;
                this->acceleration += normalisedFleeingdirection;
            }
        }
        else
        {
            for (auto& boid : boids)
            {
                totalPosition += boid.position;
            }

            sf::Vector2f centerOfFlock = totalPosition / static_cast<float>(boids.size());
            sf::Vector2f steeringDirection = centerOfFlock - position;
            float magnitudeOfSteeringDirection = sqrt(steeringDirection.x*steeringDirection.x + steeringDirection.y*steeringDirection.y);
            sf::Vector2f normalisedSteeringDirectionForce = steeringDirection / magnitudeOfSteeringDirection;

            normalisedSteeringDirectionForce *= 0.05f;
            acceleration += normalisedSteeringDirectionForce;

            for (auto& neighbourBoid : boids)
            {
                if (this == &neighbourBoid)
                {
                    continue;
                }

                float dx = (this->position.x - neighbourBoid.position.x);
                float dy = (this->position.y - neighbourBoid.position.y);
                float distance = sqrt((dx * dx) + (dy * dy));

                if (distance < 25.f)
                {
                    sf::Vector2f steerAwayDirection = this->position - neighbourBoid.position;
                    float magnitudeOfSteerAwayDirection = sqrt(steerAwayDirection.x * steerAwayDirection.x + steerAwayDirection.y * steerAwayDirection.y);
                    sf::Vector2f normalisedSteerAwayDirection = steerAwayDirection / magnitudeOfSteerAwayDirection;
                    normalisedSteerAwayDirection *= 0.05f;
                    this->acceleration += normalisedSteerAwayDirection;
                }
            }

            for (auto& boid : boids)
            {
                totalVelocity += boid.velocity;
            }

            sf::Vector2f averageVelocity = totalVelocity / static_cast<float>(boids.size());
            sf::Vector2f velocitySteering = averageVelocity - this->velocity;
            float magnitudeOfVelocitySteering = sqrt(velocitySteering.x * velocitySteering.x + velocitySteering.y * velocitySteering.y);
            if (magnitudeOfSteeringDirection > 0)
            {
                sf::Vector2f normalisedVelocitySteering = velocitySteering / magnitudeOfVelocitySteering;
                normalisedVelocitySteering *= 0.05f;
                this->acceleration += normalisedVelocitySteering;
            }
        }

        const float accelerationMagnitude = sqrt((this->acceleration.x * this->acceleration.x) + (this->acceleration.y * this->acceleration.y));

        if (accelerationMagnitude > 0.5f)
        {
            this->acceleration = (this->acceleration / accelerationMagnitude) * 0.5f;
        }

    }
};

int main()
{
    const float windowHeight = 600.f;
    const float windowWidth = 800.f;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> distributionX(0, windowWidth);
    std::uniform_real_distribution<float> distributionY(0, windowHeight);


    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boid Simulation");
    window.setFramerateLimit(60);
    window.setVerticalSyncEnabled(true);

    std::vector<Boid> boidList;

    for (int i=0; i<50; i++)
    {
        const float boidXPosition = distributionX(generator);
        const float boidYPosition = distributionY(generator);
        Boid newBoid(boidXPosition, boidYPosition, generator);
        boidList.push_back(newBoid);
    }

    std::vector<sf::CircleShape> starField;
    std::uniform_real_distribution<float> starDistributionX(0, windowWidth);
    std::uniform_real_distribution<float> starDistributionY(0, windowHeight);
    std::uniform_real_distribution<float> starRadius(1.f, 1.5f);

    for (int i=0; i<200; i++)
    {
        sf::CircleShape newStar;

        float starXPosition = starDistributionX(generator);
        float starYPosition = starDistributionY(generator);
        float radiusOfStar = starRadius(generator);

        newStar.setPosition(starXPosition, starYPosition);
        newStar.setRadius(radiusOfStar);

        starField.push_back(newStar);
    }


    bool blackHoleActive = false;
    sf::Vector2f blackHolePosition;

    sf::CircleShape blackHoleEffect(30.f);
    blackHoleEffect.setFillColor(sf::Color::Black);
    blackHoleEffect.setOutlineColor(sf::Color(144, 199, 240));
    blackHoleEffect.setOutlineThickness(2.f);
    blackHoleEffect.setOrigin(30.f, 30.f);

    sf::RectangleShape accretionDisk;
    accretionDisk.setSize(sf::Vector2f(100.f, 3.f));
    accretionDisk.setFillColor(sf::Color(42, 101, 145));
    accretionDisk.setOrigin(50.f, 1.f);
    float accretionDiskRotation = 0.f;

    while (window.isOpen())
    {
        sf::Event event;


        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    blackHoleActive = true;
                    blackHolePosition = sf::Vector2f(event.mouseButton.x, event.mouseButton.y);

                }
            }

            if (event.type == sf::Event::MouseButtonReleased)
            {
                blackHoleActive = false;
            }

        }


        sf::Vector2i pixelPosition = sf::Mouse::getPosition(window);
        sf::Vector2f mousePosition = window.mapPixelToCoords(pixelPosition);

        sf::RectangleShape fadeRect(sf::Vector2f(windowWidth, windowHeight));
        fadeRect.setFillColor(sf::Color(0, 0, 0, 70));

        window.draw(fadeRect);

        if (blackHoleActive)
        {
            blackHoleEffect.setPosition(blackHolePosition);
            accretionDisk.setPosition(blackHolePosition);
            accretionDiskRotation += 0.5f;
            accretionDisk.setRotation(accretionDiskRotation);
            window.draw(blackHoleEffect);
            window.draw(accretionDisk);
        }

        for (auto& star : starField)
        {
            window.draw(star);
        }

        for (auto& currentBoid : boidList)
        {
            currentBoid.flockRules(boidList, mousePosition, blackHoleActive, blackHolePosition);
            currentBoid.updateBoidPosition(windowWidth, windowHeight);
            window.draw(currentBoid.shape);
        }

        window.display();


    }

    return 0;

}